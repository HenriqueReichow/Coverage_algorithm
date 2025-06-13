import subprocess

N = 40

for k in range(13,N):
    print(f"--- Iniciando simulação {k} ---")
    result = subprocess.run(["python3", "simulation_mv.py", str(k)])
    if result.returncode != 0:
        print(f"Simulação {k} falhou com código {result.returncode}")
    else:
        print(f"Simulação {k} finalizada com sucesso")