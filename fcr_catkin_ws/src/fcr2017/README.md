----este pacote foi pego e modificado para realizar o primeiro trabalho de fcr, o autor original dele autorizou a utilizar esse pacote por fins educacionais na materia fundamentos computacionais de robotica.

FCR 2017
========

Pacote do ROS para o curso de Fundamentos Computacionais de Robótica - CIC - UnB

* Pacote criado e testado no Ubuntu 14.04 e ROS Indigo

* Antes de usar o pacote instale o p2os-msgs:

$ sudo apt-get install ros-indigo-p2os-msgs

================

Entrada
-------
leitura de sensores do robo odometria/laser
ponto objetivo

Saida
-----
movimento do robo com desvio de obstaculos e mapeamendo do caminho

Dependencias do ROS
-------------------
*  roscpp
*  td_msgs
*  geometry_msgs
*  sensor_msgs
*  p2os_msgs
*  tf

Dependencias fora do ROS
------------------------


Algoritmo
---------
O algoritmo implementado foi implmentado atraves de estrutura feita ja pelos monitores da materia FCR foi acrecentado o codigo e feito os trabalhos no mesmo pacote.

Início
    enquanto verdade e nao chegou pro ponto objetivo
	se nao tem obstaculo 
		vai pro ponto objetivo mapeando o caminho se nao tiver passado antes
	se tem
		desvie
    fim enquanto
fim

Descrição dos arquivos
----------------------
 
CMakeLists.txt: Arquivo de configuração da build deste pacote
package.xml: Arquivo de configuração de dependecias deste pacote e informações de versão, autor e descrição
README.md: Este arquivo

