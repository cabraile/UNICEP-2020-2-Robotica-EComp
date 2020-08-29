# 1. O Terminal Resumido

## 1.1 Tabela de Comandos

| **Comando**               | **Ação**                                       |
|---------------------------|:------------------------------------------------|
| `pwd`                     | Mostra o caminho para o diretório atual        |
| `ls`                      | Mostra os arquivos e pastas do diretório atual |
| `cd <nome da pasta>`      | Entra na pasta                                 |
| `mkdir <nome da pasta>`   | Cria uma pasta                                 |
| `touch <nome do arquivo>` | Cria um arquivo                                |
| `kate <nome do arquivo>`  | Editor de texto com interface gráfica          |
| `vim <nome do arquivo>`   | Editor de texto no terminal                    |
| `source <nome do arquivo>`| Coloca no contexto do terminal atual as definições feitas no arquivo |

## 1.2 Exemplo - Criar uma Pasta de Trabalho e Arquivos

Suponha que você abriu o terminal. Você verá algo como:
`user@ubuntu:~$ `

Isto indica que o terminal foi iniciado como usuário `user` na máquina `ubuntu` e na pasta `~`. A pasta `~` é uma abreviação de `/home/user`, então é a sua pasta atual. Assim, se você digitar `pwd`, você vai ver `/home/user`.

Se você escrever `ls` vão aparecer algumas pastas, como `Pictures`, `Documents`, `Downloads`, entre outras.

Você resolve então criar um novo diretório na pasta `Documents` chamada `arquivos_de_aula`, tem duas formas:
1. Você digita `mkdir Documents/arquivos_de_aula`. Assim você cria a pasta `arquivos_de_aula` em `Documents` mas ainda continua na pasta `~`. Para entrar na pasta `Documents`, use `cd Documents`.
2. Você navegar até a pasta `Documents` por meio de `cd Documents` e então criar a pasta `mkdir arquivos_de_aula`. Assim você já está na pasta `~/Documents`.

Navegue até a pasta `arquivos_de_aula` por meio de `cd arquivos_de_aula`.

Então você decide implementar um código em C++ para a aula de robótica e, pra isso, primeiro cria um arquivo `touch exercicio.cpp`.

Pra editar o arquivo você digita `kate exercicio.cpp`.

# 2. Usando o ROS

## 2.1 Tabela de comandos do ROS

| **Comando**                           | **Ação**                                                                                             |
|---------------------------------------|:------------------------------------------------------------------------------------------------------|
| `roscore`                             | Inicia o ROS na sua máquina.                                                                         |
| `rosrun <pacote> <nome do nó>`        | Roda um nó ROS do pacote `<pacote>`                                                                  |
| `roslaunch <pacote> <arquivo>.launch` | Roda um conjunto de nós e serviços do pacote `<pacote>` do arquivo `<arquivo>.launch`                |
| `rviz`                                | Programa de visualização de dados do ROS                                                             |
| `rqt_graph`                           | Exibe o gráfico de nós e tópicos do ROS, ou seja, todos os nós e por quais tópicos eles se comunicam |


## 2.2 Exemplo - Rodando o Gazebo e Nós do ROS

O primeiro passo é indicar para a sua máquina que ela vai virar um servidor ROS por meio de `roscore`. Para executar os outros comandos abra um novo terminal, já que esse vai ficar bloqueado.

Na máquina virtual da MathWorks, eles disponibilizam a simulação do `Turtlebot`. Para rodar o simulador `Gazebo` com a simulação do `TurtleBot`, use `roslaunch turtlebot_gazebo turtlebot_mw_office.launch`. Neste comando, o que vai acontecer: o pacote `turtlebot_gazebo` possui o arquivo `turtlebot_mw_office.launch` que possui o conjunto dos nós e serviços que fazem parte da simulação. Neste momento, a janela do `Gazebo` vai aparecer com o mundo da simulação. Lembrando: aquilo é o que a gente vê do mundo do robô, o robô não necessariamente sabe de tudo que tem neste mundo.

Se você quiser rodar o conjunto de serviços e nós que fazem teleoperação, rode em outro terminal `roslaunch turtlebot_teleop keyboard_teleop.launch`. Este terminal vai ficar bloqueado mas você poderá controlar o robô por meio das teclas `i`, `k`, `j` e `l`.

Para ver o mundo com as informações que o robô tem acesso, você pode usar o `rviz`. Você pode ver mais detalhes de como usar o `rviz` no vídeo de aula.

## 2.3 Tabela de comandos do catkin
| **Comando**                                                        | **Ação**                                          |
|--------------------------------------------------------------------|:---------------------------------------------------|
| `catkin_make`                                                      | Compila o Workspace que você está                 |
| `catkin_create_pkg <nome do pacote> <dependencia1> <dependencia2>` | Cria um pacote ROS dadas as dependências passadas |

## 2.4 Exemplo - Criando um pacote; compilando e rodando um nó do ROS

Abra o terminal, crie uma pasta onde você irá compilar seus projetos do ROS em um lugar que você prefira. Pra este exemplo, eu criei um pasta `Workspace` e nela criei outra chamada `aulas_ws`.

Dentro da pasta `aulas_ws`, crie uma pasta chamada `src`, que vai conter os nossos projetos do ROS e rode no terminal (ainda na pasta `aulas_ws`): `catkin_make`. Ele irá criar duas pastas na pasta raíz do nosso workspace, a `build/` e `devel/`. Não se preocupe com elas agora.

Abra outro terminal e entre na pasta `src` que criamos na raíz do workspace (`aulas_ws`). Dentro da pasta `src` execute o comando `catkin_create_pkg aulas_praticas roscpp std_msgs geometry_msgs sensor_msgs`. Ele irá criar uma pasta dentro de `src` com o nome `aulas_praticas` e que já está com o `CMakeLists.txt` (arquivo para compilar usando CMake) e `package.xml` (arquivo com informações do projeto e suas dependências) configurados. Como colocamos como dependência o `roscpp`, ele vai criar uma pasta `src` e uma `include` em `aulas_praticas`. 

Na pasta `aulas_praticas/src` (ou seja, na pasta `src` que está em `aulas_praticas`) colocaremos o código dos nossos nós em C++. Para cada nó gerado, precisamos alterar no `aulas_praticas/CMakeLists.txt` conforme no vídeo da aula.

Depois de editado seu arquivo do nó, na raíz do workspace `aulas_ws` use `catkin_make` para compilar o código. Caso haja algum erro, ele irá indicar qual arquivo e qual erro ocorreu. 

Caso seja compilado com sucesso, podemos usar o executável do nó no ambiente (vamos upor nesse exemplo que seja `aula1_node`) da seguinte forma:
1. Primeiro precisamos usar `source devel/setup.bash`
    * Neste comando, o seu terminal vai colocar no ambiente dele a posição de todos os executáveis compilados pelo catkin no seu projeto, o que vai fazer com que possamos usar o projeto e seus nós `rosrun` como se fosse uma variável global
2. Em seguida usamos `rosrun aulas_praticas aula1_node`
    * Aqui nós invocamos nosso nó `aula1_node` do pacote `aulas_praticas`, o qual irá interagir com outros nós do ROS.
