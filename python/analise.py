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
        
        # --- CORREÇÃO INSERIDA AQUI ---
        # Bloco para normalizar os nomes das colunas e evitar o KeyError.
        # Isto torna o script mais robusto a diferentes formatos de cabeçalho.
        rename_map = {
            'tempo_s': 'tempo',
            'setpoint_cm': 'setpoint',
            'distancia_cm': 'distancia',
            # Adicione outras variações de nome de coluna aqui se necessário
        }
        # Renomeia as colunas encontradas no mapeamento
        df.rename(columns={k: v for k, v in rename_map.items() if k in df.columns}, inplace=True)

        # Renomeia a coluna de 'distancia' para 'atual' para o resto do script
        if 'distancia' in df.columns:
            df = df.rename(columns={'distancia': 'atual'})
        
        # Verifica se as colunas essenciais existem após a tentativa de renomear
        required_cols = ['tempo', 'setpoint', 'atual']
        if not all(col in df.columns for col in required_cols):
            print(f"Erro: O ficheiro '{caminho_ficheiro}' não contém as colunas necessárias.")
            print(f"Colunas necessárias: {required_cols}. Colunas encontradas: {df.columns.tolist()}")
            return
        # --- FIM DA CORREÇÃO ---

    except FileNotFoundError:
        print(f"Erro: O ficheiro '{caminho_ficheiro}' não foi encontrado.")
        return
    except Exception as e:
        print(f"Ocorreu um erro inesperado ao ler o ficheiro: {e}")
        return

    # Encontrar os índices onde o setpoint muda
    # Usamos .diff().fillna(1) para garantir que a primeira linha seja sempre contada como uma mudança
    indices_mudanca = df[df['setpoint'].diff().fillna(1).ne(0)].index.tolist()
    
    # Se não houver mudanças, não há o que analisar
    if not indices_mudanca:
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
        # O valor inicial é o valor 'atual' na linha anterior à mudança de setpoint
        valor_inicial = df['atual'].iloc[idx_inicio_degrau - 1] if idx_inicio_degrau > 0 else dados_degrau['atual'].iloc[0]
        tempo_inicial = dados_degrau['tempo'].iloc[0]

        # Evitar analisar transições onde o valor já está no setpoint
        if setpoint_alvo == valor_inicial:
            continue

        # --- Calcular Métricas para esta transição ---
        
        # Erro em Regime Permanente
        # Usar os últimos 20% dos dados do degrau para calcular o valor estável
        pontos_regime = dados_degrau.tail(max(1, int(len(dados_degrau) * 0.20)))
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
        
        sobressinal_percent = (desvio_max / amplitude_degrau) * 100 if amplitude_degrau > 1e-6 else 0
        sobressinal_percent = max(0, sobressinal_percent)

        # Tempo de Transição (Subida/Descida) de 10% a 90%
        limite_10 = valor_inicial + 0.1 * (setpoint_alvo - valor_inicial)
        limite_90 = valor_inicial + 0.9 * (setpoint_alvo - valor_inicial)
        
        tempo_10, tempo_90 = np.nan, np.nan
        if setpoint_alvo > valor_inicial: # Subida
            tempo_10_series = dados_degrau[dados_degrau['atual'] >= limite_10]['tempo']
            tempo_90_series = dados_degrau[dados_degrau['atual'] >= limite_90]['tempo']
        else: # Descida (limites invertidos)
            tempo_10_series = dados_degrau[dados_degrau['atual'] <= limite_10]['tempo']
            tempo_90_series = dados_degrau[dados_degrau['atual'] <= limite_90]['tempo']

        if not tempo_10_series.empty:
            tempo_10 = tempo_10_series.iloc[0]
        if not tempo_90_series.empty:
            tempo_90 = tempo_90_series.iloc[0]

        tempo_transicao = abs(tempo_90 - tempo_10) if pd.notna(tempo_10) and pd.notna(tempo_90) else float('nan')

        # Tempo de Acomodação
        faixa_superior = setpoint_alvo * (1 + tolerancia_acomodacao)
        faixa_inferior = setpoint_alvo * (1 - tolerancia_acomodacao)
        
        # Encontra todos os pontos que saíram da faixa de tolerância após o início do degrau
        fora_da_faixa = dados_degrau[
            (dados_degrau['atual'] > faixa_superior) | 
            (dados_degrau['atual'] < faixa_inferior)
        ]
        
        tempo_acomodacao = float('nan')
        if not fora_da_faixa.empty:
            # O tempo de acomodação é o tempo do último ponto que estava fora da faixa
            tempo_ultimo_fora = fora_da_faixa['tempo'].max()
            tempo_acomodacao = tempo_ultimo_fora - tempo_inicial
        else:
            # Se nenhum ponto saiu da faixa, o sistema acomodou-se instantaneamente
            tempo_acomodacao = 0.0


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
    pior_tempo_transicao = np.nanmax([r['tempo_transicao'] for r in resultados_transicoes if pd.notna(r['tempo_transicao'])])
    pior_tempo_acomodacao = np.nanmax([r['tempo_acomodacao'] for r in resultados_transicoes if pd.notna(r['tempo_acomodacao'])])

    print("\n" + "="*50)
    print("      RESUMO DE DESEMPENHO (PIORES CASOS)       ")
    print("="*50)
    print(f"Pior Erro em Regime Permanente: {pior_erro:.3f} cm")
    print(f"Máximo Sobressinal / Undershoot: {pior_sobressinal:.2f} %")
    print(f"Maior Tempo de Transição (10%-90%): {pior_tempo_transicao:.3f} s")
    print(f"Maior Tempo de Acomodação (±{tolerancia_acomodacao*100:.1f}%): {pior_tempo_acomodacao:.3f} s")
    print("="*50)


# --- Executar a análise completa ---
if __name__ == "__main__":
    # Substitua 'saida_pid.csv' pelo nome do seu ficheiro se for diferente
    analisar_simulacao_completa('saida_tl.csv')
