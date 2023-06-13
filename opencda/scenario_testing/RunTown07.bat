@ECHO OFF
ECHO Beginning scenario 'custom_single_town07_cosim'
call activate opencda
C:\Users\dsmith3502\Anaconda3\envs\opencda\python C:\Apps\OpenCDA\opencda.py -t custom_single_town07_cosim -v 0.9.12 --apply_ml
C:\Users\dsmith3502\Anaconda3\envs\opencda\python C:\Apps\OpenCDA\opencda.py -t custom_single_town10_cosim -v 0.9.12 --apply_ml
call conda deactivate
PAUSE