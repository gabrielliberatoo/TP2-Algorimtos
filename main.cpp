#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

using namespace std;
const long long INF = numeric_limits<long long>::max();

struct Vertice {
    int id;
    long long distancia;
    vector<pair<Vertice*, pair<long long, long long>>> vizinhos; // (vizinho, (peso, ano))
};

struct Grafo {
    vector<Vertice*> vertices;
};

struct Aresta {
    int origem;
    int destino;
    long long peso;
    long long ano;
    long long custo;

    bool operator<(const Aresta& outra) const {
        return custo < outra.custo;
    }
};

void dijkstra(Grafo* grafo, Vertice* origem) {
    // Inicializa a distância de todos os vértices como infinito
    for (size_t i = 0; i < grafo->vertices.size(); i++) {
        if (grafo->vertices[i]) {
            grafo->vertices[i]->distancia = INF;
        }
    }

    origem->distancia = 0;

    // Fila de prioridade para armazenar os vértices a serem visitados
    priority_queue<pair<long long, Vertice*>, vector<pair<long long, Vertice*>>, greater<pair<long long, Vertice*>>> fila;
    fila.push(make_pair(0, origem));

    // Loop para visitar todos os vértices
    while (!fila.empty()) {
        Vertice* vertice = fila.top().second; // Pega o vértice com menor distância
        long long distanciaAtual = fila.top().first; // Pega a distância do vértice
        fila.pop(); // Remove o vértice da fila

        // Se a distância atual for maior que a distância do vértice, então não é necessário visitar
        if (distanciaAtual > vertice->distancia) continue;

        // Para cada vizinho do vértice, atualiza a distância se for menor
        for (const auto& vizinho_info : vertice->vizinhos) {
            Vertice* vizinho = vizinho_info.first;
            long long peso = vizinho_info.second.first;
            long long novaDistancia = vertice->distancia + peso;

            if (novaDistancia < vizinho->distancia) {
                vizinho->distancia = novaDistancia;
                fila.push(make_pair(novaDistancia, vizinho));
            }
        }
    }
}

// Calcula o valor de A1 percorrendo as distâncias finais após Dijkstra
long long calcularA1(Grafo* grafo) {
    long long A1 = 0;
    for (size_t i = 1; i < grafo->vertices.size(); ++i) {
        for (const auto& vizinho_info : grafo->vertices[i]->vizinhos) {
            if (vizinho_info.first->distancia == grafo->vertices[i]->distancia + vizinho_info.second.first) {
                A1 = max(A1, vizinho_info.second.second);
            }
        }
    }
    return A1;
}

bool todasAcessiveis(Grafo* grafo, long long anoLimite) {
    vector<bool> visitado(grafo->vertices.size(), false);
    queue<Vertice*> fila;

    fila.push(grafo->vertices[1]);
    visitado[1] = true;
    int visitados = 1;

    while (!fila.empty()) {
        Vertice* vertice = fila.front();
        fila.pop();

        for (const auto& vizinho_info : vertice->vizinhos) {
            Vertice* vizinho = vizinho_info.first;
            long long ano = vizinho_info.second.second;
            if (!visitado[vizinho->id] && ano <= anoLimite) {
                visitado[vizinho->id] = true;
                fila.push(vizinho);
                visitados++;
            }
        }
    }
    return visitados == grafo->vertices.size() - 1;
}

long long determinarAno(Grafo* grafo, long long maxAno){
    long long esquerdo = 1, direito = maxAno, resultado = maxAno;
    
    while(esquerdo <= direito){
        long long meio = (esquerdo + direito) / 2;
        if(todasAcessiveis(grafo, meio)){
            resultado = meio;
            direito = meio - 1;
        } else {
            esquerdo = meio + 1;
        }
    }
    return resultado;
}

int find(vector<int>& pai, int i){
    if (pai[i] != i){
        pai[i] = find(pai, pai[i]);
    }
    return pai[i];
} 

void unionSets(vector<int>& pai, vector<int>& rank, int x, int y){
    int raizX = find(pai, x);
    int raizY = find(pai, y);

    if(raizX != raizY){
        if(rank[raizX] > rank[raizY]){
            pai[raizY] = raizX;
        } else if (rank[raizX] < rank[raizY]){
            pai[raizX] = raizY;
        } else {
            pai[raizY] = raizX;
            rank[raizX]++;
        }
    }
}

long long kruskal(vector<Aresta>& arestas, int N){
    sort(arestas.begin(), arestas.end());

    vector<int> pai(N+1);
    vector<int> rank(N+1, 0);
    for(int i = 1; i <= N; i++){
        pai[i] = i;
    }

    long long custoTotal = 0;
    for (const auto& aresta : arestas){
        int raizU = find(pai, aresta.origem);
        int raizV = find(pai, aresta.destino);
        
        if(raizU != raizV){
            custoTotal += aresta.custo;
            unionSets(pai, rank, raizU, raizV);
        }
    }
    return custoTotal;
}

int main() {
    int N, M;
    cin >> N >> M;

    Grafo grafo;
    grafo.vertices.resize(N + 1); // O +1 é para lidar com o índice 1-based

    long long maxAno = 0;
    vector<Aresta> arestas;

    // Inicialização dos vértices
    for (int i = 1; i <= N; ++i) {
        grafo.vertices[i] = new Vertice();
        grafo.vertices[i]->id = i;
    }

    // Leitura das conexões
    for (int i = 0; i < M; ++i) {
        int u, v;
        long long a, l, c;
        cin >> u >> v >> a >> l >> c;
        maxAno = max(maxAno, a);

        grafo.vertices[u]->vizinhos.push_back({grafo.vertices[v], {l, a}});
        grafo.vertices[v]->vizinhos.push_back({grafo.vertices[u], {l, a}});

        arestas.push_back({u, v, l, a, c}); // Adiciona aresta à lista de arestas para Kruskal
    }

    // Executa o algoritmo de Dijkstra a partir do palácio real (considerado como vértice de origem)
    dijkstra(&grafo, grafo.vertices[1]);

    // Calcula o valor de A1 percorrendo as distâncias finais
    long long A1 = calcularA1(&grafo);

    // Calcula o valor de A2
    long long A2 = determinarAno(&grafo, maxAno);

    // Calcular a árvore geradora mínima
    long long menorCusto = kruskal(arestas, N);

    // Impressão das distâncias mínimas para cada vila
    for (int i = 1; i <= N; ++i) {
        cout << (grafo.vertices[i]->distancia == INF ? -1 : grafo.vertices[i]->distancia) << endl;
    }

    // Impressão dos valores de A1 e A2
    cout << A1 << endl;
    cout << A2 << endl;
    cout << menorCusto << endl;

    // Limpeza de memória
    for (int i = 1; i <= N; ++i) {
        delete grafo.vertices[i];
    }

    return 0;
}
