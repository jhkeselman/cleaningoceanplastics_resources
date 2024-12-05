#!/bin/bash
#SBATCH -N 1                   # 
#SBATCH -n 8                   # 
#SBATCH --mem=8g               # 
#SBATCH -J "Cleaning Ocean Plastics Training"   # 
#SBATCH -p short               # 
#SBATCH -t 12:00:00            # 
#SBATCH --gres=gpu:2           # 
#SBATCH -C "A100|V100"         #

module load python             #
python3 -m venv myenv
source myenv/bin/activate
pip install ultralytics
pip install numpy==1.23.0

python turing1_train.py       #