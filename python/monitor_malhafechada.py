import sys
import time
import threading
import re
import serial
import matplotlib.pyplot as plt
import csv

# --- Listas para armazenar os dados ---
timestamps = []
setpoints = []
actuals = []
control_signals = []
running = True

# --- Cria um Lock para sincronizar o acesso às listas ---
data_lock = threading.Lock()

# --- Função para ler os dados da porta serial ---
def read_serial(ser):
    """
    Lê continuamente a porta serial em uma thread separada.
    Usa um lock para adicionar dados às listas de forma segura.
    """
    global running
    print("Thread de leitura iniciada. Aguardando dados do Arduino...")
    while running:
        try:
            # Tenta ler uma linha da porta serial
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # --- DEBUG: Imprime tudo que chega da serial ---
            if line:
                print(f"[SERIAL] {line}")

            # Expressão regular para extrair os 4 valores da string
            # Ex: "Tempo:12.345, Setpoint:15.00, Distancia:14.98, SinalControle_RPM:5.12"
            match = re.search(r"Tempo:([-\d.]+), Setpoint:([-\d.]+), Distancia:([-\d.]+), SinalControle_RPM:([-\d.]+)", line)
            
            if match:
                # --- DEBUG: Informa que a linha foi reconhecida ---
                print(" -> Formato reconhecido. Processando dados.")
                
                # Extrai os 4 valores capturados pela regex
                t, sp, ac, cs = map(float, match.groups())
                
                # Usa o Lock para garantir a escrita atômica nas listas
                with data_lock:
                    timestamps.append(t)
                    setpoints.append(sp)
                    actuals.append(ac)
                    control_signals.append(cs)
            elif line:
                # --- DEBUG: Informa que a linha não foi reconhecida ---
                print(" -> Formato NÃO reconhecido.")

        except serial.SerialException as e:
            print(f"Erro de leitura serial: {e}. Encerrando...")
            running = False
        except Exception as e:
            print(f"Erro inesperado na leitura serial: {e}")
            running = False

# --- Função para plotar os dados em tempo real ---
def plot_realtime():
    """
    Plota os dados em tempo real em uma janela do Matplotlib.
    """
    global running
    
    plt.ion()
    fig, ax1 = plt.subplots(figsize=(12, 7))
    fig.canvas.manager.set_window_title('Monitoramento em Tempo Real do Controle em Malha Fechada')
    
    ax2 = ax1.twinx()

    while running:
        with data_lock:
            # Copia os dados para evitar mantê-los travados durante a plotagem
            ts_copy = list(timestamps)
            sp_copy = list(setpoints)
            ac_copy = list(actuals)
            cs_copy = list(control_signals)

        if ts_copy:
            ax1.clear()
            ax2.clear()

            ax1.plot(ts_copy, sp_copy, 'r--', label='Setpoint (cm)')
            ax1.plot(ts_copy, ac_copy, 'b-', linewidth=2, label='Distância Atual (cm)')
            ax1.set_xlabel('Tempo (s)')
            ax1.set_ylabel('Distância (cm)', color='b')
            ax1.tick_params(axis='y', labelcolor='b')
            ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
            
            ax2.plot(ts_copy, cs_copy, 'g:', label='Sinal de Controle (RPM)')
            ax2.set_ylabel('Sinal de Controle (RPM)', color='g')
            ax2.tick_params(axis='y', labelcolor='g')
            
            lines1, labels1 = ax1.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

            plt.title('Desempenho do Controlador')
            fig.tight_layout()
        
        try:
            plt.pause(0.1)
            # Verifica se a janela de plotagem ainda está aberta
            if not plt.fignum_exists(fig.number):
                print("Janela de plotagem fechada. Encerrando...")
                running = False
        except Exception:
            # Captura outros erros que podem ocorrer ao fechar a janela
            running = False
            break
            
    plt.ioff()
    print("Loop de plotagem encerrado.")

# --- Função principal ---
def main():
    global running
    
    port = sys.argv[1] if len(sys.argv) > 1 else input("Porta serial (ex: COM3 ou /dev/ttyUSB0): ")
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Conectado à porta {port} com baud rate {baud}.")
    except serial.SerialException as e:
        print(f"Erro ao abrir a porta serial {port}: {e}")
        sys.exit(1)

    print("Aguardando 2 segundos para o Arduino inicializar...")
    time.sleep(2)
    ser.flushInput() # Limpa qualquer dado residual na porta serial
    
    read_thread = threading.Thread(target=read_serial, args=(ser,))
    read_thread.daemon = True
    read_thread.start()

    plot_realtime()

    # --- Bloco de Encerramento ---
    running = False
    print("Sinal de encerramento enviado para a thread de leitura.")
    read_thread.join(timeout=2)
    
    if ser.is_open:
        ser.close()
        print("Porta serial fechada.")

    if timestamps:
        output_filename = 'saida_malhafechada.csv'
        with open(output_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['tempo', 'setpoint', 'distancia', 'sinal_controle_rpm'])
            data_to_save = zip(timestamps, setpoints, actuals, control_signals)
            writer.writerows(data_to_save)
        print(f'Leitura finalizada. Dados salvos em {output_filename}')
    else:
        print("Nenhum dado foi coletado para salvar.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupção do usuário (Ctrl+C). Encerrando...")
        running = False
