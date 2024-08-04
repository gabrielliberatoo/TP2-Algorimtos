#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;
const int INF = numeric_limits<int>::max();

struct Vertice {
    int id;
    int ano;
    int distancia;
    int custo;
    vector<pair<Vertice*, pair<int, int>>> vizinhos; // (vizinho, (peso, ano))
};

struct Grafo {
    vector<Vertice*> vertices;
};

void dijkstra(Grafo* grafo, Vertice* origem, int& A1, int& A2) {
    // Inicializa a distância de todos os vértices como infinito
    for (int i = 0; i < grafo->vertices.size(); i++) {
        if (grafo->vertices[i]) {
            grafo->vertices[i]->distancia = INF;
        }
    }

    origem->distancia = 0;
    A1 = 0;
    A2 = -1; // Inicializa A2 como -1

    // Fila de prioridade para armazenar os vértices a serem visitados
    priority_queue<pair<int, Vertice*>, vector<pair<int, Vertice*>>, greater<pair<int, Vertice*>>> fila;
    fila.push(make_pair(0, origem));

    // Vetor para verificar a conectividade
    vector<bool> alcançado(grafo->vertices.size(), false);
    alcançado[origem->id] = true;

    // Loop para visitar todos os vértices
    while (!fila.empty()) {
        Vertice* vertice = fila.top().second; // Pega o vértice com menor distância
        int distanciaAtual = fila.top().first; // Pega a distância do vértice
        fila.pop(); // Remove o vértice da fila

        // Se a distância atual for maior que a distância do vértice, então não é necessário visitar
        if (distanciaAtual > vertice->distancia) continue;

        // Para cada vizinho do vértice, atualiza a distância e o ano se for menor
        for (const auto& [vizinho, info] : vertice->vizinhos) {
            int peso = info.first;
            int ano = info.second;
            int novaDistancia = vertice->distancia + peso;

            if (novaDistancia < vizinho->distancia) {
                vizinho->distancia = novaDistancia;
                A1 = max(A1, ano); // Atualiza A1 com o maior ano encontrado até agora
                fila.push(make_pair(novaDistancia, vizinho));
            }
            // Marca o vizinho como alcançado
            alcançado[vizinho->id] = true;
        }
    }

    // Determina o ano A2, onde todos os vértices foram alcançados
    for (int i = 1; i < alcançado.size(); ++i) {
        if (!alcançado[i]) {
            A2 = -1; // Se algum vértice não foi alcançado
            return;
        }
    }
    A2 = A1; // Se todos os vértices foram alcançados, A2 é o mesmo que A1
}

int main() {
    int N, M;
    cin >> N >> M;

    Grafo grafo;
    grafo.vertices.resize(N + 1); // O +1 é para lidar com o índice 1-based

    // Inicialização dos vértices
    for (int i = 1; i <= N; ++i) {
        grafo.vertices[i] = new Vertice();
        grafo.vertices[i]->id = i;
    }

    // Leitura das conexões
    for (int i = 0; i < M; ++i) {
        int u, v, a, l, c;
        cin >> u >> v >> a >> l >> c;

        grafo.vertices[u]->vizinhos.push_back({grafo.vertices[v], {l, a}});
        grafo.vertices[v]->vizinhos.push_back({grafo.vertices[u], {l, a}});
    }

    // Executa o algoritmo de Dijkstra a partir do palácio real (considerado como vértice de origem)
    int A1, A2;
    dijkstra(&grafo, grafo.vertices[1], A1, A2);

    // Impressão das distâncias mínimas para cada vila
    for (int i = 1; i <= N; ++i) {
        cout << (grafo.vertices[i]->distancia == INF ? -1 : grafo.vertices[i]->distancia) << endl;
    }

    // Impressão dos valores de A1 e A2
    cout << A1 << endl;
    cout << A2 << endl;

    // Limpeza de memória
    for (int i = 1; i <= N; ++i) {
        delete grafo.vertices[i];
    }

    return 0;
}

