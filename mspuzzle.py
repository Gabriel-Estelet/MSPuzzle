# Código gerado com ajuda do Gemini 2.5 Pro

import heapq

# --- Configuração ---

# Um dicionário "NEIGHBORS" mapeando cada índice da grade para seus índices vizinhos válidos.
# Isso representa os movimentos possíveis para a peça vazia (número 9).

NEIGHBORS = {
    0: [1, 3], 1: [0, 2, 4], 2: [1, 5],
    3: [0, 4, 6], 4: [1, 3, 5, 7], 5: [2, 4, 8],
    6: [3, 7], 7: [4, 6, 8], 8: [5, 7]
}

# --- Função Heurística ---

def calculate_manhattan_distance(state, goal_state):
    """"
    Calcula a distância de Manhattan para um estado dado.

    A distância de Manhattan é a soma das distâncias horizontais e verticais
    de cada peça em relação à sua posição alvo. É uma heurística "admissível"
    porque nunca superestima o custo real (número de movimentos) para alcançar o objetivo.

    Args:
        state (tuple): A configuração atual do quebra-cabeça.
        goal_state (tuple): A configuração alvo do quebra-cabeça.

    Returns:
        int: O valor heurístico calculado (h-score).
    """

    h_score = 0
    for i in range(9):
        current_val = state[i]
        if current_val != 9 and current_val != goal_state[i]:
            # Pega as coordenadas (x, y) atuais da peça
            current_x, current_y = i % 3, i // 3
            # Pega o índice da peça na configuração alvo
            goal_idx = goal_state.index(current_val)
            goal_x, goal_y = goal_idx % 3, goal_idx // 3
            
            # Adiciona a distância ao h_score total
            h_score += abs(current_x - goal_x) + abs(current_y - goal_y)
    return h_score

# --- Implementação do algoritmo A* ---

def a_star_search(start_state, goal_state):
    """
    Faz a busca A* para encontrar o caminho mais curto do estado inicial ao estado final (solução).

    Args:
        start_state (tuple): A configuração inicial do quebra-cabeça.
        goal_state (tuple): A configuração alvo do quebra-cabeça.

    Returns:
        list or None: Uma lista de estados do quebra-cabeça representando o caminho mais curto,
                      ou None se nenhum caminho for encontrado.                                                  
    """

    # A fila de prioridade para os nós a serem visitados. Armazena (f_score, estado).
    open_set = [(0, start_state)] 

    # dicionário came_from para reconstruir o caminho. Mapeia estado -> estado_pai.
    came_from = {start_state: None}

    # g_score: Custo do início até o nó atual. Mapeia estado -> custo.
    g_score = {start_state: 0}
    
    # f_score: Custo total estimado (g_score + h_score). Mapeia estado -> custo.
    h_start = calculate_manhattan_distance(start_state, goal_state)
    f_score = {start_state: h_start}

    while open_set:
        # Pegue o nó com o menor f_score da fila de prioridade
        _, current_state = heapq.heappop(open_set)

        # Se alcançado o objetivo, reconstruímos e retornamos o caminho
        if current_state == goal_state:
            path = []
            while current_state:
                path.append(list(current_state)) # Converte tuple de volta para lista para saída
                current_state = came_from[current_state]
            return path[::-1] # Reverte o caminho para obter a ordem início -> objetivo

        # --- Gerar Vizinhos (Estados Sucessores) ---
        idx_9 = current_state.index(9)
        
        for neighbor_idx in NEIGHBORS[idx_9]:
            # Crie o novo estado trocando 9 com seu vizinho
            new_state_list = list(current_state)
            new_state_list[idx_9], new_state_list[neighbor_idx] = new_state_list[neighbor_idx], new_state_list[idx_9]
            neighbor_state = tuple(new_state_list)
            
            # tentative_g_score é a distância do início até o vizinho passando pelo atual
            tentative_g_score = g_score[current_state] + 1

            # Se esse caminho para o vizinho for melhor que qualquer outro anterior, registre-o
            if tentative_g_score < g_score.get(neighbor_state, float('inf')):
                came_from[neighbor_state] = current_state
                g_score[neighbor_state] = tentative_g_score
                h_score = calculate_manhattan_distance(neighbor_state, goal_state)
                f_score[neighbor_state] = tentative_g_score + h_score
                
                # Adicione o vizinho ao conjunto aberto para avaliação
                heapq.heappush(open_set, (f_score[neighbor_state], neighbor_state))

    return None # Retorna None se o conjunto aberto for esgotado e nenhuma solução for encontrada

# --- Programa Principal ---

if __name__ == "__main__":
    # Configurações inicial e alvo (usando tuplas, pois são hashable para chaves de dicionário)
    # Inserir configuração inicial do quebra-cabeça aqui:
    p_start = (3, 7, 5, 6, 9, 4, 2, 1, 8)
    # Resolução do quebra-cabeça
    p_end = (1, 2, 3, 4, 5, 6, 7, 8, 9)

    print(f"Iniciando configuração: {list(p_start)}")
    print(f"Configuração alvo: {list(p_end)}")
    print("\nResolvendo com busca A*...\n")

    solution_path = a_star_search(p_start, p_end)

    print("\n" + "="*30)
    if solution_path:
        print(f"✅ Resolvido! Encontrado melhor solução com {len(solution_path) - 1} passos.")
        print("="*30)
        for i, step in enumerate(solution_path):
            print(f"Passo {i}: {step}")
    else:
        print("❌ Solução não encontrada.")
    print("="*30)

    input("\nPressione Enter para sair...")    