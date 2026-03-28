from pathlib import Path
from run_slam import run_slam

dataset_version = "V2.4.F"
param_dir: Path = Path(__file__).parent.parent / "params" / f"hercules_{dataset_version}"
run_slam(str(param_dir), None, f'ROMAN - HERCULES - {dataset_version} - FINAL', None, False, None, False, False, False)