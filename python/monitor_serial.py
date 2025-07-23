import sys
import time
import threading
import re
import serial
import matplotlib.pyplot as plt
import csv

# --- Listas globais para armazenar os dados ---
timestamps = []
setpoints = []
actuals = []
velocities = []
running = True

# --- Cria um Lock para sincronizar o acesso às listas ---
# Isso previne erros de "race condition" entre a thread de leitura e a de plotagem.
data_lock = threading.Lock()

# --- Função para ler os dados da porta serial ---
def read_serial(ser):
    """
    Lê continuamente a porta serial em uma thread separada.
    Usa um lock para adicionar dados às listas de forma segura.
    """
    global running
    while running:
        try:
            # Tenta ler uma linha da porta serial
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # Expressão regular para extrair os 4 valores da string
                # Ex: "Tempo:12.345, Setpoint:15.00, Distancia:14.98, RPM:5.12"
                match = re.search(r"Tempo:([-\d.]+), Setpoint:([-\d.]+), Distancia:([-\d.]+), RPM:([-\d.]+)", line)
                
                if match:
                    # Extrai os 4 valores capturados pela regex
                    t, sp, ac, vel = map(float, match.groups())
                    
                    # --- Usa o Lock para garantir a escrita atômica nas listas ---
                    with data_lock:
                        timestamps.append(t)
                        setpoints.append(sp)
                        actuals.append(ac)
                        velocities.append(vel)
                else:
                    # Imprime a linha se ela não corresponder ao formato esperado (para depuração)
                    if line and "Inicializando" not in line and "Iniciado" not in line:
                        print(f"Linha não reconhecida: {line}")
            else:
                # Pequena pausa para não sobrecarregar a CPU quando não há dados
                time.sleep(0.05)

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
    Usa um lock para copiar os dados das listas de forma segura.
    Encerra o programa de forma limpa quando a janela do gráfico é fechada.
    """
    global running
    
    # --- ALTERAÇÃO PRINCIPAL: Função para ser chamada quando a janela do gráfico for fechada ---
    def on_close(event):
        """Handler para o evento de fechamento da janela do gráfico."""
        global running
        print("Janela de plotagem fechada pelo usuário. Iniciando encerramento...")
        running = False

    plt.ion() # Ativa o modo interativo
    fig, ax1 = plt.subplots(figsize=(12, 7))
    fig.canvas.manager.set_window_title('Monitoramento em Tempo Real do Controle PID')
    
    # --- ALTERAÇÃO PRINCIPAL: Conecta o evento de fechamento da janela à nossa função on_close ---
    fig.canvas.mpl_connect('close_event', on_close)

    # Cria um segundo eixo Y que compartilha o mesmo eixo X
    ax2 = ax1.twinx()

    # O loop continua enquanto 'running' for True. 
    # A função on_close definirá running=False quando a janela for fechada.
    while running:
        ts_copy, sp_copy, ac_copy, vel_copy = [], [], [], []
        
        # --- Copia os dados de forma segura usando o lock ---
        with data_lock:
            if timestamps: # Só copia se houver dados
                ts_copy = list(timestamps)
                sp_copy = list(setpoints)
                ac_copy = list(actuals)
                vel_copy = list(velocities)

        # Só tenta plotar se houver dados para evitar erros
        if ts_copy:
            ax1.clear()
            ax2.clear()

            # Plotar Distância e Setpoint no eixo Y primário (ax1)
            ax1.plot(ts_copy, sp_copy, 'r--', label='Setpoint (cm)')
            ax1.plot(ts_copy, ac_copy, 'b-', linewidth=2, label='Distância Atual (cm)')
            ax1.set_xlabel('Tempo (s)')
            ax1.set_ylabel('Distância (cm)', color='b')
            ax1.tick_params(axis='y', labelcolor='b')
            ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
            
            # Plotar RPM no eixo Y secundário (ax2)
            ax2.plot(ts_copy, vel_copy, 'g:', label='RPM do Motor')
            ax2.set_ylabel('RPM', color='g')
            ax2.tick_params(axis='y', labelcolor='g')
            
            # Unificar as legendas de ambos os eixos em um único local
            lines1, labels1 = ax1.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

            plt.title('Desempenho do Controlador PID')
            fig.tight_layout() # Ajusta o plot para evitar sobreposição de labels
        
        try:
            # Pausa para permitir que a GUI atualize e a leitura continue.
            plt.pause(0.1) 
        except Exception:
            # Se a janela for fechada, o backend da GUI pode ser destruído,
            # causando um erro em plt.pause(). O 'running' já será False,
            # então podemos simplesmente sair do loop.
            if not running:
                break
    
    plt.ioff() # Desativa o modo interativo
    plt.close(fig) # --- ALTERAÇÃO: Fecha a janela do gráfico explicitamente para limpeza
    print("Função de plotagem encerrada.")

# --- Função principal para iniciar as threads e gerenciar o programa ---
def main():
    global running
    
    # --- Configuração da Porta Serial ---
    port = sys.argv[1] if len(sys.argv) > 1 else input("Porta serial (ex: COM3 ou /dev/ttyUSB0): ")
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

    ser = None
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Conectado à porta {port} com baud rate {baud}.")
    except serial.SerialException as e:
        print(f"Erro ao abrir a porta serial {port}: {e}")
        print("Verifique se a porta não está em uso por outro programa e se você tem permissão.")
        sys.exit(1)

    # Aguarda a inicialização da conexão serial e do Arduino
    print("Aguardando 2 segundos para a inicialização do dispositivo serial...")
    time.sleep(2)
    
    # Inicia a thread de leitura da serial
    read_thread = threading.Thread(target=read_serial, args=(ser,))
    read_thread.daemon = True # Permite que o programa principal saia mesmo se a thread estiver rodando
    read_thread.start()

    # Inicia a função de plotagem na thread principal.
    # Esta função agora bloqueará a execução até que a janela do gráfico seja fechada.
    plot_realtime()

    # --- Bloco de Encerramento ---
    # Este código executa quando plot_realtime() retorna (após a janela ser fechada).
    # 'running' já foi definido como False pelo evento de fechamento da janela.
    print("Aguardando a thread de leitura finalizar...")
    read_thread.join(timeout=2) # Aguarda a thread de leitura terminar
    
    if read_thread.is_alive():
        print("Aviso: A thread de leitura não encerrou no tempo esperado.")

    if ser and ser.is_open:
        ser.close()
        print("Porta serial fechada.")

    # Salva os dados coletados em um arquivo CSV
    if timestamps:
        output_filename = 'saida_tl.csv' # Alterado para .csv para maior compatibilidade
        try:
            with open(output_filename, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['tempo_s', 'setpoint_cm', 'distancia_cm', 'rpm'])
                # Usa zip para garantir que os dados de cada linha correspondam
                with data_lock: # Bloqueia os dados uma última vez para garantir consistência
                    data_to_save = zip(timestamps, setpoints, actuals, velocities)
                    writer.writerows(data_to_save)
            print(f'Leitura finalizada. Dados salvos em "{output_filename}"')
        except IOError as e:
            print(f"Erro ao salvar o arquivo: {e}")
    else:
        print("Nenhum dado foi coletado para salvar.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupção do usuário (Ctrl+C). Encerrando...")
        running = False
    print("Programa finalizado.")
