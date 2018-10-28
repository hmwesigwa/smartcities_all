import os
import sys
import numpy as np

folder = '/home/hushiji/Research/smart_cities/smart_cities/survey_experiments/realdata'

def alter_palmetto_file(seed):
    
    str1 = """#!/bin/bash      
#PBS -N example      
#PBS -l select=1:ncpus=16:mem=60gb:interconnect=1g,walltime=72:00:00  
#PBS -j oe      

source /etc/profile.d/modules.sh  
module purge   
cd /home/hushiji/Research/smart_cities/smart_cities/survey_experiments/realdata


"""
    pal_FILE= open('palmetto.txt','w')
    pal_FILE.write(str1)
    str2 = 'python solve_model.py ' + str(seed) 
    pal_FILE.write(str2)
    pal_FILE.close()



def submit_jobs():
    myseeds = np.genfromtxt('myseeds.txt', dtype = int)
    for seed in myseeds:
        alter_palmetto_file(seed)
        command = folder + '/submitall.sh'
        os.system(command)


if __name__ == '__main__':
    submit_jobs()
