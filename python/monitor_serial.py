# serial_plot.py
# Leitura em tempo real da saída serial do Arduino e plot dinâmico, com setpoint enviado via Python

import sys
import time
import threading
import re

import serial
import matplotlib.pyplot as plt

# Parâmetros iniciais
port = sys.argv[1] if len(sys.argv) > 1 else input("Porta serial (ex: COM3 ou /dev/ttyUSB0): ")
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

# Abrir conexão serial
try:
    ser = serial.Serial(port, baud, timeout=1)
except serial.SerialException as e:
    print(f"Erro ao abrir a porta serial {port}: {e}")
    print("Verifique se a porta não está em uso por outro programa e se você tem permissão.")
    sys.exit(1)

# Aguarda reset do Arduino
time.sleep(2)

# Estruturas para armazenar dados
timestamps = []        # tempo relativo
setpoints = []         # x_des
actuals = []           # x_hat
velocities = []        # x_hat velocidade
running = True
start_time = time.time()

# Função de leitura serial
def read_serial():
    global running
    while running:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            # Extrai todos os números da linha
            nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
            if len(nums) >= 3:
                t_rel = time.time() - start_time
                sp, ac, vel = map(float, nums[:3])
                timestamps.append(t_rel)
                setpoints.append(sp)
                actuals.append(ac)
                velocities.append(vel)
        except serial.SerialException as e:
            print(f"Erro de leitura serial: {e}")
            running = False # Encerra a thread se houver um erro de serial
        except Exception as e:
            print(f"Erro inesperado na leitura serial: {e}")
            running = False

# Função para enviar setpoint
def send_setpoint():
    global running
    while running:
        try:
            # Solicita o novo setpoint ao usuário
            new_setpoint_str = input("Digite o novo setpoint (cm) ou 'q' para sair: ")
            if new_setpoint_str.lower() == 'q':
                running = False
                break
            
            try:
                new_setpoint = float(new_setpoint_str)
                if 0 < new_setpoint < 400: # Validação básica do setpoint
                    ser.write(f"{new_setpoint}\n".encode('utf-8'))
                    print(f"Setpoint enviado: {new_setpoint} cm")
                else:
                    print("Setpoint inválido. Digite um valor entre 0 e 400.")
            except ValueError:
                print("Entrada inválida. Por favor, digite um número ou 'q'.")
        except Exception as e:
            print(f"Erro ao enviar setpoint: {e}")
            running = False # Encerra a thread em caso de erro

# Função de plot em tempo real
def plot_realtime():
    plt.ion() # Ativa o modo interativo
    fig, ax = plt.subplots()
    fig.canvas.manager.set_window_title('Monitoramento em Tempo Real do Arduino') # Título da janela do gráfico
    
    # setpoint_line, = ax.plot([], [], 'r--', label='Setpoint Desejado') # Esta linha não é mais necessária aqui, pois plotamos o setpoint recebido

    while running:
        if timestamps:
            ax.clear()
            ax.plot(timestamps, setpoints, label='Setpoint Recebido')
            ax.plot(timestamps, actuals, label='Atual')
            ax.plot(timestamps, velocities, label='Velocidade')
            
            ax.set_xlabel('Tempo (s)')
            ax.set_ylabel('Velocidade')
            ax.legend(loc='upper right')
            ax.grid(True) # Adiciona grade ao gráfico
            plt.pause(0.05) # Pequena pausa para atualização do gráfico
        else:
            # Se não houver dados, ainda precisamos de uma pequena pausa para evitar loop ocupado
            plt.pause(0.05)
    
    # Desativa o modo interativo e mostra o gráfico final ao sair do loop
    plt.ioff()
    plt.show()

# Threads
read_thread = threading.Thread(target=read_serial)
send_thread = threading.Thread(target=send_setpoint)

# Inicia as threads de leitura e envio
read_thread.start()
send_thread.start()

# Inicia o plot na thread principal
try:
    plot_realtime() # plot_realtime agora roda na thread principal
except KeyboardInterrupt:
    print("\nInterrupção do usuário. Encerrando threads...")
finally:
    running = False # Sinaliza para todas as threads pararem
    
    # Garante que as threads sejam finalizadas
    read_thread.join(timeout=1) # Adiciona timeout para evitar travamento
    send_thread.join(timeout=1)
    
    if ser.is_open:
        ser.close()
        print("Porta serial fechada.")

    # Gravação em arquivo
    if timestamps: # Garante que há dados para salvar
        with open('saida.txt', 'w') as f:
            f.write('tempo,setpoint,atual,velocidade\n')
            for t, sp, ac, vel in zip(timestamps, setpoints, actuals, velocities):
                f.write(f"{t:.3f},{sp:.6f},{ac:.6f},{vel:.6f}\n")
        print('Leitura finalizada. Dados salvos em saida.txt')
    else:
        print("Nenhum dado foi coletado para salvar.")

