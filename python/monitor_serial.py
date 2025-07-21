import sys
import time
import threading
import re
import serial
import matplotlib.pyplot as plt

# --- Configuração da Porta Serial ---
# Pede a porta ao usuário se não for fornecida como argumento
port = sys.argv[1] if len(sys.argv) > 1 else input("Porta serial (ex: COM3 ou /dev/ttyUSB0): ")
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

try:
    ser = serial.Serial(port, baud, timeout=1)
except serial.SerialException as e:
    print(f"Erro ao abrir a porta serial {port}: {e}")
    print("Verifique se a porta não está em uso por outro programa e se você tem permissão.")
    sys.exit(1)

# Aguarda a inicialização da conexão serial
time.sleep(2)

# --- Listas para armazenar os dados ---
timestamps = []
setpoints = []
actuals = []
velocities = []
running = True

# --- Função para ler os dados da porta serial ---
def read_serial():
    global running
    while running:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            # Regex atualizado para extrair os valores dos rótulos corretos
            match = re.search(r"Tempo:([-\d.]+), Setpoint:([-\d.]+), Distancia:([-\d.]+), RPM:([-\d.]+)", line)
            
            if match:
                # Extrai os 4 valores capturados pela regex
                t, sp, ac, vel = map(float, match.groups())
                
                timestamps.append(t)
                setpoints.append(sp)
                actuals.append(ac)
                velocities.append(vel)
            else:
                # Imprime a linha se ela não corresponder ao formato esperado (para depuração)
                if line:
                    print(f"Linha não reconhecida: {line}")

        except serial.SerialException as e:
            print(f"Erro de leitura serial: {e}")
            running = False
        except Exception as e:
            print(f"Erro inesperado na leitura serial: {e}")
            running = False

# --- Função para plotar os dados em tempo real ---
def plot_realtime():
    plt.ion()
    fig, ax1 = plt.subplots(figsize=(12, 6))
    fig.canvas.manager.set_window_title('Monitoramento em Tempo Real do Controle PID')
    
    # Cria um segundo eixo Y que compartilha o mesmo eixo X
    ax2 = ax1.twinx()

    while running:
        if timestamps:
            ax1.clear()
            ax2.clear()

            # Plotar Distância e Setpoint no eixo Y primário (ax1)
            ax1.plot(timestamps, setpoints, 'r--', label='Setpoint (cm)')
            ax1.plot(timestamps, actuals, 'b-', label='Distância Atual (cm)')
            ax1.set_xlabel('Tempo (s)')
            ax1.set_ylabel('Distância (cm)', color='b')
            ax1.tick_params(axis='y', labelcolor='b')
            ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
            
            # Plotar RPM no eixo Y secundário (ax2)
            ax2.plot(timestamps, velocities, 'g:', label='RPM do Motor')
            ax2.set_ylabel('RPM', color='g')
            ax2.tick_params(axis='y', labelcolor='g')
            
            # Unificar as legendas de ambos os eixos em um único local
            lines1, labels1 = ax1.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

            plt.title('Desempenho do Controlador PID')
            plt.pause(0.05)
        else:
            plt.pause(0.05)

    plt.ioff()
    plt.show()

# --- Função principal para iniciar as threads ---
def main():
    global running
    
    read_thread = threading.Thread(target=read_serial)
    read_thread.start()

    try:
        plot_realtime()
    except KeyboardInterrupt:
        print("\nInterrupção do usuário. Encerrando...")
    finally:
        running = False
        read_thread.join(timeout=2)
        
        if ser.is_open:
            ser.close()
            print("Porta serial fechada.")

        if timestamps:
            with open('saida_pid.csv', 'w') as f:
                f.write('tempo,setpoint,distancia,rpm\n')
                for t, sp, ac, vel in zip(timestamps, setpoints, actuals, velocities):
                    f.write(f"{t:.3f},{sp:.2f},{ac:.2f},{vel:.2f}\n")
            print('Leitura finalizada. Dados salvos em saida_pid.csv')
        else:
            print("Nenhum dado foi coletado para salvar.")

if __name__ == "__main__":
    main()
