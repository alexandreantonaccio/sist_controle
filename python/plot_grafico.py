import pandas as pd
import matplotlib.pyplot as plt

def plotar_grafico_completo(caminho_ficheiro, nome_saida='grafico_completo_pid.png'):
    """
    Lê os dados de saída de um controlador e gera um gráfico de linha completo.

    Args:
        caminho_ficheiro (str): O caminho para o ficheiro .txt ou .csv.
        nome_saida (str): O nome do ficheiro de imagem a ser salvo.
    """
    try:
        # Carregar os dados do ficheiro
        df = pd.read_csv(caminho_ficheiro)
        # Renomear colunas para consistência, se necessário
        if 'distancia' in df.columns:
            df = df.rename(columns={'distancia': 'atual'})

        # --- Gerar o Gráfico ---
        plt.style.use('seaborn-v0_8-whitegrid')
        fig, ax = plt.subplots(figsize=(15, 7))

        ax.plot(df['tempo'], df['atual'], label='Valor Atual', color='cornflowerblue')
        ax.plot(df['tempo'], df['setpoint'], label='Setpoint', color='red', linestyle='--')

        ax.set_title('Resposta Completa do Controlador PID', fontsize=16)
        ax.set_xlabel('Tempo (s)', fontsize=12)
        ax.set_ylabel('Posição (cm)', fontsize=12)
        ax.legend(loc='best')
        ax.set_xlim(left=0) # Iniciar o gráfico do tempo 0

        plt.tight_layout()
        plt.savefig(nome_saida)
        print(f"Gráfico '{nome_saida}' gerado com sucesso.")

    except FileNotFoundError:
        print(f"Erro: O ficheiro '{caminho_ficheiro}' não foi encontrado.")
    except Exception as e:
        print(f"Ocorreu um erro inesperado: {e}")

# --- Executar a função de plotagem ---
if __name__ == "__main__":
    # Substitua 'saida_pid.txt' pelo nome do seu ficheiro se for diferente
    plotar_grafico_completo('saida.txt')
