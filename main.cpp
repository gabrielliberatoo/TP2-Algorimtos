#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

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

struct Aresta {
    int origem;
    int destino;
    int peso;
    int ano;
    int custo;

    bool operator<(const Aresta& outra) const {
        return custo < outra.custo;
    }
};

void dijkstra(Grafo* grafo, Vertice* origem, int& A1) {
    // Inicializa a distância de todos os vértices como infinito
    for (int i = 0; i < grafo->vertices.size(); i++) {
        if (grafo->vertices[i]) {
            grafo->vertices[i]->distancia = INF;
        }
    }

    origem->distancia = 0;
    A1 = 0;

    // Fila de prioridade para armazenar os vértices a serem visitados
    priority_queue<pair<int, Vertice*>, vector<pair<int, Vertice*>>, greater<pair<int, Vertice*>>> fila;
    fila.push(make_pair(0, origem));

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
        }
    }
}

// Função para verificar se todas as vilas são acessíveis a partir do palácio real até um determinado ano
bool todasAcessiveis(Grafo* grafo, int anoLimite) {
    vector<bool> visitado(grafo->vertices.size(), false);
    queue<Vertice*> fila;

    fila.push(grafo->vertices[1]);
    visitado[1] = true;
    int visitados = 1;

    while (!fila.empty()) {
        Vertice* vertice = fila.front();
        fila.pop();

        for (const auto& [vizinho, info] : vertice->vizinhos) {
            int ano = info.second;
            if (!visitado[vizinho->id] && ano <= anoLimite) {
                visitado[vizinho->id] = true;
                fila.push(vizinho);
                visitados++;
            }
        }
    }
    return visitados == grafo->vertices.size() - 1;
}

int determinarAno(Grafo* grafo, int maxAno){
    int esquerdo = 1, direito = maxAno, resultado = maxAno;
    
    while(esquerdo<=direito){
        int meio = (esquerdo+direito)/2;
        if(todasAcessiveis(grafo, meio)){
            resultado = meio;
            direito = meio-1;
    } else {
        esquerdo = meio+1;
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

int kruskal(vector<Aresta>& arestas, int N){
    sort(arestas.begin(), arestas.end());

    vector<int> pai(N+1);
    vector<int> rank(N+1, 0);
    for(int i = 1; i <= N; i++){
        pai[i] = i;
    }

    int custoTotal = 0;
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

    int maxAno = 0;
    vector<Aresta> arestas;

    // Inicialização dos vértices
    for (int i = 1; i <= N; ++i) {
        grafo.vertices[i] = new Vertice();
        grafo.vertices[i]->id = i;
    }

    // Leitura das conexões
    for (int i = 0; i < M; ++i) {
        int u, v, a, l, c;
        cin >> u >> v >> a >> l >> c;
        maxAno = max(maxAno, a);

        grafo.vertices[u]->vizinhos.push_back({grafo.vertices[v], {l, a}});
        grafo.vertices[v]->vizinhos.push_back({grafo.vertices[u], {l, a}});

        arestas.push_back({u, v, l, a, c});
    }

    // Executa o algoritmo de Dijkstra a partir do palácio real (considerado como vértice de origem)
    int A1;
    dijkstra(&grafo, grafo.vertices[1], A1);

    // Calcula o valor de A2
    int A2 = determinarAno(&grafo, maxAno);

    //calcular arvore geradora minima
    int menorCusto = kruskal(arestas, N);

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
