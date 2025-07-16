import sys
import time
import threading
import re

import serial
import matplotlib.pyplot as plt

port = sys.argv[1] if len(sys.argv) > 1 else input("Porta serial (ex: COM3 ou /dev/ttyUSB0): ")
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600


try:
    ser = serial.Serial(port, baud, timeout=1)
except serial.SerialException as e:
    print(f"Erro ao abrir a porta serial {port}: {e}")
    print("Verifique se a porta não está em uso por outro programa e se você tem permissão.")
    sys.exit(1)

time.sleep(2)


timestamps = []        
setpoints = []         
actuals = []           
velocities = []        
running = True
start_time = time.time()

def read_serial():
    global running
    while running:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
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
            running = False 
        except Exception as e:
            print(f"Erro inesperado na leitura serial: {e}")
            running = False

def send_setpoint():
    global running
    while running:
        try:
            new_setpoint_str = input("Digite o novo setpoint (cm) ou 'q' para sair: ")
            if new_setpoint_str.lower() == 'q':
                running = False
                break
            
            try:
                new_setpoint = float(new_setpoint_str)
                if 0 < new_setpoint < 400: 
                    ser.write(f"{new_setpoint}\n".encode('utf-8'))
                    print(f"Setpoint enviado: {new_setpoint} cm")
                else:
                    print("Setpoint inválido. Digite um valor entre 0 e 400.")
            except ValueError:
                print("Entrada inválida. Por favor, digite um número ou 'q'.")
        except Exception as e:
            print(f"Erro ao enviar setpoint: {e}")
            running = False 

def plot_realtime():
    plt.ion() 
    fig, ax = plt.subplots()
    fig.canvas.manager.set_window_title('Monitoramento em Tempo Real do Arduino') 
    
    

    while running:
        if timestamps:
            ax.clear()
            ax.plot(timestamps, setpoints, label='Setpoint Recebido')
            ax.plot(timestamps, actuals, label='Atual')
            ax.plot(timestamps, velocities, label='Velocidade')
            
            ax.set_xlabel('Tempo (s)')
            ax.set_ylabel('Velocidade')
            ax.legend(loc='upper right')
            ax.grid(True) 
            plt.pause(0.05) 
        else:
            plt.pause(0.05)
    

    plt.ioff()
    plt.show()


read_thread = threading.Thread(target=read_serial)
send_thread = threading.Thread(target=send_setpoint)


read_thread.start()
send_thread.start()


try:
    plot_realtime() 
except KeyboardInterrupt:
    print("\nInterrupção do usuário. Encerrando threads...")
finally:
    running = False 
    
    read_thread.join(timeout=1) 
    send_thread.join(timeout=1)
    
    if ser.is_open:
        ser.close()
        print("Porta serial fechada.")

    if timestamps: 
        with open('saida_pid.txt', 'w') as f:
            f.write('tempo,setpoint,atual,velocidade\n')
            for t, sp, ac, vel in zip(timestamps, setpoints, actuals, velocities):
                f.write(f"{t:.3f},{sp:.6f},{ac:.6f},{vel:.6f}\n")
        print('Leitura finalizada. Dados salvos em saida.txt')
    else:
        print("Nenhum dado foi coletado para salvar.")

