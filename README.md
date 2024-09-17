# shark-mb-ros

Esses parâmetros pertencem à configuração do Trajectory Scoring do DWA (Dynamic Window Approach) local planner no ROS1. Esses parâmetros influenciam como as trajetórias candidatas são avaliadas e escolhidas para que o robô siga um caminho seguro e eficiente.

Vamos analisar cada parâmetro com exemplos:
1. path_distance_bias

    Descrição: Define o peso que o controlador dá para seguir o caminho gerado pelo planejador global. Um valor mais alto faz o robô priorizar permanecer no caminho global, enquanto um valor mais baixo faz com que ele seja mais flexível em se desviar desse caminho.
    Valor padrão: 64.0
    Exemplo:
        Se path_distance_bias = 64.0: O robô se esforçará muito para ficar no caminho planejado pelo global planner, mesmo que isso signifique passar mais perto de obstáculos.
        Se path_distance_bias = 10.0: O robô dará menos importância ao caminho global, podendo escolher trajetórias alternativas se isso melhorar o custo em termos de distância ao objetivo ou evitar obstáculos.

2. goal_distance_bias

    Descrição: Define o peso dado à aproximação do objetivo. Um valor mais alto faz o robô priorizar movimentos diretos em direção ao objetivo, enquanto um valor mais baixo dá mais flexibilidade para seguir outras trajetórias.
    Valor padrão: 24.0
    Exemplo:
        Se goal_distance_bias = 24.0: O robô vai priorizar se aproximar do objetivo diretamente, mesmo que precise desviar um pouco do caminho global.
        Se goal_distance_bias = 5.0: O robô será menos inclinado a seguir diretamente para o objetivo, preferindo outras trajetórias que possam ser mais seguras ou eficientes, como seguir o caminho global com mais precisão.

3. occdist_scale

    Descrição: Controla o peso dado à distância de obstáculos. Um valor mais alto faz com que o robô seja mais cauteloso ao evitar obstáculos, priorizando trajetórias que o mantenham longe deles.
    Valor padrão: 0.02
    Exemplo:
        Se occdist_scale = 0.02: O robô será relativamente cuidadoso ao evitar obstáculos, priorizando trajetórias que mantenham uma distância segura deles.
        Se occdist_scale = 0.1: O robô será muito mais cauteloso, dando grande peso à distância de obstáculos e evitando-os mesmo que isso signifique desviar muito do caminho ou do objetivo.

4. forward_point_distance

    Descrição: Define a distância à frente do robô onde um ponto adicional é colocado para fins de pontuação da trajetória. Esse ponto adicional ajuda a determinar a suavidade da trajetória.
    Valor padrão: 0.325 metros.
    Exemplo:
        Se forward_point_distance = 0.325: O robô avalia as trajetórias considerando um ponto 32.5 cm à frente de sua posição atual, o que pode ajudar a escolher trajetórias mais suaves e com menor variação.
        Se forward_point_distance = 0.1: O ponto adicional será colocado mais perto do robô, e ele considerará trajetórias mais curtas e com mudanças bruscas de direção, o que pode levar a movimentos mais rápidos, mas menos suaves.

5. stop_time_buffer

    Descrição: Especifica a quantidade de tempo que o robô deve parar antes de colidir com um obstáculo. Isso ajuda a garantir que o robô tenha tempo suficiente para frear.
    Valor padrão: 0.2 segundos.
    Exemplo:
        Se stop_time_buffer = 0.2: O robô calculará as trajetórias de modo que tenha pelo menos 0.2 segundos de folga antes de colidir com um obstáculo, garantindo que haja tempo suficiente para parar.
        Se stop_time_buffer = 1.0: O robô será muito mais conservador, garantindo que tenha 1 segundo de folga antes de colidir, o que pode resultar em trajetórias mais lentas e seguras.

6. scaling_speed

    Descrição: Define a velocidade absoluta (em metros por segundo) a partir da qual o tamanho do footprint do robô começa a ser escalonado (ou aumentado). Isso é útil em velocidades mais altas, onde o robô precisa ser mais cauteloso devido à sua menor capacidade de fazer curvas fechadas.
    Valor padrão: 0.25 metros/segundo.
    Exemplo:
        Se scaling_speed = 0.25: Quando o robô estiver se movendo a 0.25 m/s ou mais rápido, seu footprint será escalado para levar em conta a menor capacidade de evitar obstáculos em alta velocidade.
        Se scaling_speed = 1.0: O robô só começa a escalar o footprint quando estiver se movendo a 1 m/s ou mais, o que pode ser arriscado se o ambiente for apertado ou houver obstáculos próximos.

7. max_scaling_factor

    Descrição: Controla o quanto o footprint do robô será escalado em alta velocidade. Um valor mais alto faz com que o footprint aparente do robô seja maior, o que o torna mais cauteloso ao calcular trajetórias.
    Valor padrão: 0.2
    Exemplo:
        Se max_scaling_factor = 0.2: O footprint do robô pode ser escalado em até 20% de seu tamanho original em altas velocidades, tornando-o mais cauteloso ao escolher trajetórias perto de obstáculos.
        Se max_scaling_factor = 0.5: O footprint pode aumentar em até 50%, fazendo com que o robô evite ainda mais obstáculos em alta velocidade, o que pode resultar em trajetórias muito mais conservadoras.