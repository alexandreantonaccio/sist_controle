import pandas as pd
import numpy as np

def analisar_simulacao_completa(caminho_ficheiro, tolerancia_acomodacao=0.05):
    """
    Analisa a totalidade dos dados de saída de um controlador PID,
    identificando todas as transições de setpoint e reportando os piores
    valores de desempenho encontrados.

    Args:
        caminho_ficheiro (str): O caminho para o ficheiro .txt ou .csv.
        tolerancia_acomodacao (float): A percentagem de tolerância para o tempo de acomodação.
    """
    try:
        df = pd.read_csv(caminho_ficheiro)
        if 'distancia' in df.columns:
            df = df.rename(columns={'distancia': 'atual'})

    except FileNotFoundError:
        print(f"Erro: O ficheiro '{caminho_ficheiro}' não foi encontrado.")
        return
    except Exception as e:
        print(f"Ocorreu um erro inesperado ao ler o ficheiro: {e}")
        return

    # Encontrar os índices onde o setpoint muda
    indices_mudanca = df[df['setpoint'].diff().ne(0)].index.tolist()
    
    # Se não houver mudanças, não há o que analisar
    if not indices_mudanca or len(indices_mudanca) <= 1:
        print("Nenhuma transição de setpoint encontrada para analisar.")
        return

    resultados_transicoes = []

    # Analisar cada transição
    for i in range(len(indices_mudanca)):
        idx_inicio_degrau = indices_mudanca[i]
        
        # O fim do degrau é o início do próximo, ou o fim do dataframe
        idx_fim_degrau = indices_mudanca[i+1] if (i + 1) < len(indices_mudanca) else len(df)
        
        dados_degrau = df.iloc[idx_inicio_degrau:idx_fim_degrau].copy()
        
        if len(dados_degrau) < 2:
            continue

        setpoint_alvo = dados_degrau['setpoint'].iloc[0]
        valor_inicial = df['atual'].iloc[idx_inicio_degrau - 1] if idx_inicio_degrau > 0 else dados_degrau['atual'].iloc[0]
        tempo_inicial = dados_degrau['tempo'].iloc[0]

        # Evitar analisar transições inválidas
        if setpoint_alvo == valor_inicial:
            continue

        # --- Calcular Métricas para esta transição ---
        
        # Erro em Regime Permanente
        pontos_regime = dados_degrau.tail(int(len(dados_degrau) * 0.20)) # Usar 20% para mais estabilidade
        valor_final_estavel = pontos_regime['atual'].mean()
        erro_regime = abs(setpoint_alvo - valor_final_estavel)

        # Sobressinal (Overshoot) / Undershoot
        amplitude_degrau = abs(setpoint_alvo - valor_inicial)
        desvio_max = 0
        if setpoint_alvo > valor_inicial: # Subida
            valor_pico = dados_degrau['atual'].max()
            desvio_max = valor_pico - setpoint_alvo
        else: # Descida
            valor_pico = dados_degrau['atual'].min()
            desvio_max = setpoint_alvo - valor_pico
        
        sobressinal_percent = (desvio_max / amplitude_degrau) * 100 if amplitude_degrau > 0 else 0
        sobressinal_percent = max(0, sobressinal_percent)

        # Tempo de Transição (Subida/Descida)
        limite_10 = valor_inicial + 0.1 * (setpoint_alvo - valor_inicial)
        limite_90 = valor_inicial + 0.9 * (setpoint_alvo - valor_inicial)
        
        if setpoint_alvo > valor_inicial: # Subida
            tempo_10 = dados_degrau[dados_degrau['atual'] >= limite_10]['tempo'].min()
            tempo_90 = dados_degrau[dados_degrau['atual'] >= limite_90]['tempo'].min()
        else: # Descida (invertido)
            tempo_10 = dados_degrau[dados_degrau['atual'] <= limite_10]['tempo'].min()
            tempo_90 = dados_degrau[dados_degrau['atual'] <= limite_90]['tempo'].min()

        tempo_transicao = abs(tempo_90 - tempo_10) if pd.notna(tempo_10) and pd.notna(tempo_90) else float('nan')

        # Tempo de Acomodação
        limite_superior = setpoint_alvo * (1 + tolerancia_acomodacao)
        limite_inferior = setpoint_alvo * (1 - tolerancia_acomodacao)
        
        fora_da_faixa = dados_degrau[
            (dados_degrau['atual'] > limite_superior) | 
            (dados_degrau['atual'] < limite_inferior)
        ]
        
        tempo_acomodacao_abs = fora_da_faixa['tempo'].max() if not fora_da_faixa.empty else tempo_inicial
        tempo_acomodacao = tempo_acomodacao_abs - tempo_inicial if pd.notna(tempo_acomodacao_abs) else float('nan')

        resultados_transicoes.append({
            'de_setpoint': valor_inicial,
            'para_setpoint': setpoint_alvo,
            'erro_regime': erro_regime,
            'sobressinal': sobressinal_percent,
            'tempo_transicao': tempo_transicao,
            'tempo_acomodacao': tempo_acomodacao
        })

    # --- Apresentar o resumo dos piores casos ---
    if not resultados_transicoes:
        print("Não foi possível calcular métricas para nenhuma transição.")
        return

    pior_erro = max(r['erro_regime'] for r in resultados_transicoes)
    pior_sobressinal = max(r['sobressinal'] for r in resultados_transicoes)
    pior_tempo_transicao = np.nanmax([r['tempo_transicao'] for r in resultados_transicoes])
    pior_tempo_acomodacao = np.nanmax([r['tempo_acomodacao'] for r in resultados_transicoes])

    print("\n" + "="*50)
    print("      RESUMO DE DESEMPENHO (PIORES CASOS)      ")
    print("="*50)
    print(f"Pior Erro em Regime Permanente: {pior_erro:.2f} cm")
    print(f"Máximo Sobressinal / Undershoot: {pior_sobressinal:.2f} %")
    print(f"Maior Tempo de Transição (10%-90%): {pior_tempo_transicao:.2f} s")
    print(f"Maior Tempo de Acomodação (±{tolerancia_acomodacao*100:.1f}%): {pior_tempo_acomodacao:.2f} s")
    print("="*50)


# --- Executar a análise completa ---
if __name__ == "__main__":
    # Substitua 'saida_pid.txt' pelo nome do seu ficheiro se for diferente
    analisar_simulacao_completa('saida_pid.txt')
