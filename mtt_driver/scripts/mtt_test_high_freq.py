#!/usr/bin/env python3
"""
MTT-154 Unified High-Frequency Performance Test
Outil unifié pour tester le système MTT à haute fréquence avec rapports standardisés.

Usage:
  python3 mtt_test_high_freq.py --duration 60 --frequency 400
  python3 mtt_test_high_freq.py --duration 120 --frequency 200 --with-modes --report-format csv
"""

import argparse
import can
import csv
import json
import os
import psutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple


@dataclass
class TestResults:
    """Résultats standardisés du test."""
    # Configuration du test
    test_id: str
    start_time: str
    duration_s: float
    target_frequency_hz: int
    
    # Métriques CAN globales
    total_frames: int
    global_frequency_hz: float
    
    # Canal 001 (Driver MTT)
    mtt_frames_001: int
    mtt_frequency_001_hz: float
    mtt_patterns_001: int
    
    # Canal 2FF (Tachometer)
    tacho_frames_2ff: int
    tacho_frequency_2ff_hz: float
    
    # Performance système (net du monitoring)
    avg_cpu_percent: float
    max_cpu_percent: float
    avg_memory_mb: float
    max_memory_mb: float
    
    # Status
    test_status: str
    notes: str


class MTTUnifiedHighFreqTester:
    """Testeur unifié pour système MTT à haute fréquence."""
    
    def __init__(self, duration: int, target_frequency: int = 400, with_modes: bool = False, report_format: str = "json"):
        self.duration = duration
        self.target_frequency = target_frequency
        self.with_modes = with_modes
        self.report_format = report_format
        
        # Identifiant unique du test
        self.test_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.start_time = datetime.now().isoformat()
        
        # État du test
        self.running = False
        self.test_start_timestamp = 0.0
        
        # Bus CAN
        self.bus: Optional[can.BusABC] = None
        
        # Stockage des données
        self.frames_001: List[dict] = []
        self.frames_2ff: List[dict] = []
        self.system_metrics: List[dict] = []
        
        # Processus externes
        self.fake_tacho_process: Optional[subprocess.Popen] = None
        self.mtt_processes: Dict[str, psutil.Process] = {}
        self.workspace_root = Path(__file__).resolve().parents[3]
        
        # Thread de monitoring
        self.monitor_thread: Optional[threading.Thread] = None
        
        # Résultats
        self.results_dir = self.workspace_root / "data" / "performance_results"
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        # Signal handling
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Gestionnaire d'arrêt propre."""
        print(f"\n🛑 Arrêt demandé (signal {signum})")
        self.stop()
        sys.exit(0)
    
    def check_prerequisites(self) -> bool:
        """Vérifie les prérequis du système."""
        print("INFO: Vérification des prérequis...")
        
        # Vérifier vcan0
        try:
            result = subprocess.run(["ip", "link", "show", "vcan0"], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                print("ERROR: Interface vcan0 non trouvée")
                return False
        except:
            print("ERROR: Impossible de vérifier vcan0")
            return False
        
        # Vérifier ROS2
        try:
            result = subprocess.run(["ros2", "node", "list"], 
                                  capture_output=True, text=True, timeout=5)
            if "/mtt_ros_wrapper" not in result.stdout:
                print("ERROR: mtt_ros_wrapper non actif")
                return False
        except:
            print("ERROR: ROS2 non accessible")
            return False
        
        print("SUCCESS: Prérequis OK")
        return True
    
    def start_fake_tachometer(self) -> bool:
        """Démarre le fake tachometer à la fréquence cible."""
        print(f"INFO: Démarrage fake tachometer {self.target_frequency}Hz...")

        script_path = self.workspace_root / "src" / "mtt_driver" / "scripts" / "mtt_cmd_tachometer_sim.py"
        if not script_path.is_file():
            print(f"ERROR: Tachometer simulator introuvable: {script_path}")
            return False
        
        cmd = [
            sys.executable, str(script_path),
            "--can-interface", "vcan0"
        ]
        
        try:
            self.fake_tacho_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            print(f"SUCCESS: Fake tachometer démarré (PID: {self.fake_tacho_process.pid})")
            return True
        except Exception as e:
            print(f"ERROR: Erreur démarrage fake tachometer: {e}")
            return False
    
    def setup_can_monitoring(self) -> bool:
        """Configure le monitoring CAN."""
        try:
            self.bus = can.interface.Bus(
                interface='socketcan',
                channel='vcan0',
                receive_own_messages=True
            )
            print("SUCCESS: Bus CAN configuré")
            return True
        except Exception as e:
            print(f"ERROR: Erreur bus CAN: {e}")
            return False
    
    def find_mtt_processes(self):
        """Trouve les processus MTT pour monitoring CPU/mémoire."""
        current_processes = {}
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                
                if 'mtt_ros_wrapper' in cmdline:
                    current_processes['mtt_ros_wrapper'] = proc
                elif 'mtt_odometry_manager' in cmdline:
                    current_processes['mtt_odometry_manager'] = proc
                elif 'mtt_teleop' in cmdline:
                    current_processes['mtt_teleop'] = proc
                    
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        
        self.mtt_processes = current_processes
        print(f"INFO: Processus MTT trouvés: {list(current_processes.keys())}")
    
    def collect_system_metrics(self) -> dict:
        """Collecte les métriques système (sans le monitoring)."""
        # CPU/mémoire des processus MTT seulement
        mtt_cpu_total = 0.0
        mtt_memory_total = 0.0
        
        for name, proc in self.mtt_processes.items():
            try:
                cpu = proc.cpu_percent()
                memory = proc.memory_info().rss / (1024 * 1024)  # MB
                mtt_cpu_total += cpu
                mtt_memory_total += memory
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        return {
            'timestamp': time.time(),
            'mtt_cpu_percent': mtt_cpu_total,
            'mtt_memory_mb': mtt_memory_total,
            'system_cpu_percent': psutil.cpu_percent(),
            'system_memory_percent': psutil.virtual_memory().percent
        }
    
    def monitor_loop(self):
        """Boucle principale de monitoring."""
        last_metrics_time = time.time()
        
        while self.running:
            current_time = time.time()
            elapsed = current_time - self.test_start_timestamp
            
            # Arrêt automatique
            if elapsed >= self.duration:
                break
            
            try:
                # Recevoir trames CAN
                if self.bus:
                    message = self.bus.recv(timeout=0.1)
                else:
                    time.sleep(0.1)
                    continue
                if message:
                    frame_data = {
                        'timestamp': current_time,
                        'arbitration_id': message.arbitration_id,
                        'data': list(message.data),
                        'data_hex': ' '.join(f'{b:02X}' for b in message.data)
                    }
                    
                    if message.arbitration_id == 0x001:
                        self.frames_001.append(frame_data)
                    elif message.arbitration_id == 0x2FF:
                        self.frames_2ff.append(frame_data)
                
                # Métriques système (toutes les secondes)
                if current_time - last_metrics_time >= 1.0:
                    metrics = self.collect_system_metrics()
                    self.system_metrics.append(metrics)
                    last_metrics_time = current_time
                    
                    # Affichage progress
                    progress = (elapsed / self.duration) * 100
                    freq_001 = len(self.frames_001) / elapsed if elapsed > 0 else 0
                    freq_2ff = len(self.frames_2ff) / elapsed if elapsed > 0 else 0
                    
                    print(f"\\r[{elapsed:5.1f}s] {progress:5.1f}% | "
                          f"001: {freq_001:6.1f}Hz | "
                          f"2FF: {freq_2ff:6.1f}Hz | "
                          f"CPU: {metrics['mtt_cpu_percent']:4.1f}% | "
                          f"Mem: {metrics['mtt_memory_mb']:5.1f}MB", end="", flush=True)
                
            except can.CanOperationError:
                continue
            except Exception as e:
                print(f"\nERROR: Erreur monitoring: {e}")
                break
    
    def calculate_results(self) -> TestResults:
        """Calcule les résultats finaux."""
        total_frames = len(self.frames_001) + len(self.frames_2ff)
        actual_duration = time.time() - self.test_start_timestamp
        
        # Fréquences
        freq_001 = len(self.frames_001) / actual_duration if actual_duration > 0 else 0
        freq_2ff = len(self.frames_2ff) / actual_duration if actual_duration > 0 else 0
        global_freq = total_frames / actual_duration if actual_duration > 0 else 0
        
        # Patterns uniques canal 001
        unique_patterns = len(set(tuple(f['data']) for f in self.frames_001))
        
        # Métriques système
        if self.system_metrics:
            avg_cpu = sum(m['mtt_cpu_percent'] for m in self.system_metrics) / len(self.system_metrics)
            max_cpu = max(m['mtt_cpu_percent'] for m in self.system_metrics)
            avg_memory = sum(m['mtt_memory_mb'] for m in self.system_metrics) / len(self.system_metrics)
            max_memory = max(m['mtt_memory_mb'] for m in self.system_metrics)
        else:
            avg_cpu = max_cpu = avg_memory = max_memory = 0.0
        
        # Status du test
        status = "SUCCESS"
        notes = []
        
        target_min = self.target_frequency * 0.875  # 87.5% du target acceptable
        
        if freq_001 < target_min:
            status = "PARTIAL"
            notes.append(f"Fréquence 001 faible: {freq_001:.1f}Hz")
        
        if freq_2ff < target_min:
            notes.append(f"Fréquence 2FF faible: {freq_2ff:.1f}Hz")
        
        if not self.frames_001:
            status = "FAILED"
            notes.append("Aucune trame canal 001")
        
        return TestResults(
            test_id=self.test_id,
            start_time=self.start_time,
            duration_s=actual_duration,
            target_frequency_hz=self.target_frequency,
            total_frames=total_frames,
            global_frequency_hz=global_freq,
            mtt_frames_001=len(self.frames_001),
            mtt_frequency_001_hz=freq_001,
            mtt_patterns_001=unique_patterns,
            tacho_frames_2ff=len(self.frames_2ff),
            tacho_frequency_2ff_hz=freq_2ff,
            avg_cpu_percent=avg_cpu,
            max_cpu_percent=max_cpu,
            avg_memory_mb=avg_memory,
            max_memory_mb=max_memory,
            test_status=status,
            notes="; ".join(notes) if notes else "Test nominal"
        )
    
    def save_results(self, results: TestResults):
        """Sauvegarde les résultats dans le format demandé."""
        results_file = self.results_dir / f"test_{self.test_id}"
        
        if self.report_format == "csv":
            csv_file = results_file.with_suffix('.csv')
            with open(csv_file, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=asdict(results).keys())
                writer.writeheader()
                writer.writerow(asdict(results))
            print(f"INFO: Résultats sauvés: {csv_file}")
        
        else:  # JSON par défaut
            json_file = results_file.with_suffix('.json')
            with open(json_file, 'w') as f:
                json.dump(asdict(results), f, indent=2)
            print(f"INFO: Résultats sauvés: {json_file}")
        
        # Données détaillées (optionnel)
        if len(self.frames_001) > 0 or len(self.frames_2ff) > 0:
            details_file = self.results_dir / f"test_{self.test_id}_details.json"
            details = {
                'frames_001': self.frames_001[:100],  # Limiter pour la taille
                'frames_2ff': self.frames_2ff[:100],
                'system_metrics': self.system_metrics
            }
            with open(details_file, 'w') as f:
                json.dump(details, f, indent=2)
            print(f"INFO: Détails sauvés: {details_file}")
    
    def print_report(self, results: TestResults):
        """Affiche le rapport final."""
        target_min = self.target_frequency * 0.875  # 87.5% du target acceptable
        
        print(f"\\n\\n{'='*80}")
        print(f"RAPPORT MTT-154 TEST {self.target_frequency}Hz")
        print(f"{'='*80}")
        print(f"Test ID:                 {results.test_id}")
        print(f"Durée:                   {results.duration_s:.2f}s")
        print(f"Status:                  {results.test_status}")
        print(f"")
        print(f"PERFORMANCE GLOBALE:")
        print(f"  Total trames:          {results.total_frames:,}")
        print(f"  Fréquence globale:     {results.global_frequency_hz:.1f} Hz")
        print(f"")
        print(f"CANAL 001 (Driver MTT):")
        print(f"  Trames:                {results.mtt_frames_001:,}")
        print(f"  Fréquence:             {results.mtt_frequency_001_hz:.1f} Hz")
        print(f"  Patterns uniques:      {results.mtt_patterns_001:,}")
        print(f"  Cible {self.target_frequency}Hz:           {'ATTEINT' if results.mtt_frequency_001_hz >= target_min else 'MANQUÉ'}")
        print(f"")
        print(f"CANAL 2FF (Tachometer):")
        print(f"  Trames:                {results.tacho_frames_2ff:,}")
        print(f"  Fréquence:             {results.tacho_frequency_2ff_hz:.1f} Hz")
        print(f"  Cible {self.target_frequency}Hz:           {'ATTEINT' if results.tacho_frequency_2ff_hz >= target_min else 'MANQUÉ'}")
        print(f"")
        print(f"RESSOURCES SYSTÈME (MTT seulement):")
        print(f"  CPU moyen:             {results.avg_cpu_percent:.1f}%")
        print(f"  CPU max:               {results.max_cpu_percent:.1f}%")
        print(f"  Mémoire moyenne:       {results.avg_memory_mb:.1f} MB")
        print(f"  Mémoire max:           {results.max_memory_mb:.1f} MB")
        print(f"")
        if results.notes:
            print(f"NOTES: {results.notes}")
            print(f"")
        print(f"CONCLUSION:")
        if results.test_status == "SUCCESS":
            print(f"  SUCCESS: Système MTT-154 fonctionne correctement à {self.target_frequency}Hz")
        elif results.test_status == "PARTIAL":
            print(f"  WARNING: Performance partielle - voir notes")
        else:
            print(f"  ERROR: Test échoué - voir notes")
        print(f"{'='*80}")
    
    def cleanup(self):
        """Nettoyage des ressources."""
        if self.fake_tacho_process:
            try:
                os.killpg(os.getpgid(self.fake_tacho_process.pid), signal.SIGTERM)
                self.fake_tacho_process.wait(timeout=3)
            except:
                pass
        
        if self.bus:
            self.bus.shutdown()
    
    def run(self) -> TestResults:
        """Lance le test complet."""
        print(f"INFO: DÉMARRAGE TEST MTT-154 à {self.target_frequency}Hz")
        print(f"Durée: {self.duration}s | Mode tests: {self.with_modes} | Format: {self.report_format}")
        print(f"Test ID: {self.test_id}")
        
        try:
            # Prérequis
            if not self.check_prerequisites():
                raise Exception("Prérequis non satisfaits")
            
            # Configuration
            if not self.setup_can_monitoring():
                raise Exception("Impossible de configurer CAN")
            
            # Démarrage fake tachometer
            if not self.start_fake_tachometer():
                raise Exception("Impossible de démarrer fake tachometer")
            
            time.sleep(2)  # Laisser démarrer
            
            # Processus MTT
            self.find_mtt_processes()
            
            print(f"\nINFO: DÉBUT DU TEST ({self.duration}s)...")
            self.running = True
            self.test_start_timestamp = time.time()
            
            # Monitoring thread
            self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
            self.monitor_thread.start()
            
            # Attendre fin du test
            self.monitor_thread.join()
            
            print(f"\n\nINFO: Test terminé")
            
            # Calcul et sauvegarde résultats
            results = self.calculate_results()
            self.save_results(results)
            self.print_report(results)
            
            return results
            
        except KeyboardInterrupt:
            print(f"\\n🛑 Test interrompu par utilisateur")
            raise
        except Exception as e:
            print(f"\nERROR: Erreur: {e}")
            raise
        finally:
            self.running = False
            self.cleanup()
    
    def stop(self):
        """Arrêt du test."""
        self.running = False


def main():
    parser = argparse.ArgumentParser(description="MTT-154 Unified High-Frequency Performance Test")
    parser.add_argument("--duration", "-d", type=int, default=60,
                        help="Durée du test en secondes (défaut: 60)")
    parser.add_argument("--frequency", "-f", type=int, default=400,
                        help="Fréquence cible en Hz (défaut: 400)")
    parser.add_argument("--with-modes", "-m", action="store_true",
                        help="Activer les tests de changement de mode")
    parser.add_argument("--report-format", "-r", choices=["json", "csv"], default="json",
                        help="Format du rapport (défaut: json)")
    
    args = parser.parse_args()
    
    if args.duration < 10:
        print("ERROR: Durée minimum: 10 secondes")
        return 1
    
    try:
        tester = MTTUnifiedHighFreqTester(
            duration=args.duration,
            target_frequency=args.frequency,
            with_modes=args.with_modes,
            report_format=args.report_format
        )
        
        results = tester.run()
        
        # Code de sortie basé sur le status
        if results.test_status == "SUCCESS":
            return 0
        elif results.test_status == "PARTIAL":
            return 1
        else:
            return 2
            
    except KeyboardInterrupt:
        print("\\n🛑 Test annulé")
        return 130
    except Exception as e:
        print(f"\nERROR: Échec du test: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
