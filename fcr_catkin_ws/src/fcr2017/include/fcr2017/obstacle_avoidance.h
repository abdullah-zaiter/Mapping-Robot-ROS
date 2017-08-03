#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <p2os_msgs/SonarArray.h>
#include <nav_msgs/Odometry.h>    //
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <stdbool.h>

using namespace std;


#define PI 3.14159265358979323846                  //o famoso PI
#define AVOIDING_ANGLE 90                         //o angulo de abertura que o robo percebe obstaculos na frente
#define MINIMUN_DISTANCE 0.3                       //distancia minima que o robo pode chegar do obstaculo
//#define SAFETY_FACTOR_OF_ANGLE 30                //graus de seguranca para girar a mais do q valor decidido pelo codigo para desviar do obstaculo
#define ANGLE_SECURITY_FACTOR_OF_ROTATING 3.5      //uma constante para tirar o efeito da inercia do robo na hora de girar
#define CORRECTION_ANGLE_FACTOR  10                 //erro maximo de orientacao em graus

#define X1 0                                       //deficinoes para facilitar a interpretacao do codigo 
#define X2 1
#define Y1 2
#define Y2 3

//velocidade angular e linear
#define VEL_LIN 0.4
#define VEL_ANG 28

class Node;
class Edges;

/*--------------------------------------------------------------------DECLARACOES DAS FUNCOES-----------------------------------------------------------------------------*/
    void Path(int CurrentNode,int WantedNode);                            //dijkstra, que acha o melhor caminho entre dois dados nos com os pesos sendo as distancias entre os nos
    void ArrayDivider();                                                  //pegar o array das leituras do sensor laser e reorganizar ele para vetor menor
    void FuseTheAngles(int Angle, int part);                              //achar a casa que tem a menor distancia no array de distancia e atribuir o valor dela para o pedaco inteiro
    void PrintTheSensorArray(int n);                                      //printa o array das leituras do sensor laser
    void PrintGraph();                                                    //printar o grafo com os pesos dos arcos
    void WhichNode(double x, double y);                                   //testar para qual no uma coordenada pertence
    void WalkFromNodeToAnother(int StartingNode, int Target);             //fazer o robo andar de um dado no inicial ate um no final
    void SendVel(double linear, double angular);                          //facilitar o envio de velocidade
    void TurnTheRobotIfItNeedsIt(int StartingNode, int Target);           //checa se o proximo no que o robo quer ir precisa de rotacao do robo
    void SequenceOfNodesToWalk();                                         //organizar o andamento do robo entre os nos
    void Turn180degrees();                                                //girar 180 graus
    void Turn90degrees(int IsClockwise);                                  //girar 90 graus
    void Delay(double time);                                              //delay para checar o andamento do codigo
    void AddEdges(int vert, double distance);                             //para adicionar os arcos e a distancia (entre os nos) para o grafo 
    void AnalyseTheSituation();                                           //analisa o caso dos flags antes de fazer algum ato
    void PrintGrade();                                                    //printar a grade quando precisar
    void PrepareTheMatrix(int NodeNumber);                                //redimensionar a matriz no no dependendo do tamanho dela 
    void CreateAndFillAFile(int NodeNumber);                              //criar o arquivo na pasta grids
    void MakeTheGrid(int NodeNumber);                                     //ir preenchendo a grade enquanto o robo esta andando
    void DealingWithFile();                                               //organiza o andamento de mexer com os arquivos referentes a cada no
    void RectangularToPolar(double x, double y,double *r, double *theta); //mudanca de coordenada retangular para polar
    void PolarToRectangular(double r, double theta,double *x, double *y); //mudanca de coordenada polar para retangular
    void AvoidTheObstacle();                                              //desviar do obstaculo quando encontrar
    void HasBeenCorrupted();                                              //resolver o problema de comecar em ponto qualquer pedindo pro usuario o numero do no e outras informacoes
    void FixOrientation(int Axe);                                         //corrigir a orientacao do robo quando estiver errada
    void InitializeGlobalVariables();                                     //inicializar as variaveis globais
    
    bool CreatGraph();                                                    //criar grafo usando arquivo texto
    bool ThereIsAnObstacle();                                             //so pela primeira vez, guarda as ultimas orientacoes do robo antes de sair da linha
    bool ThereIsAnObstacle2();                                            //verifica se tem obstaculo sem guardar valores atuais da orientacao (no processo de desviar)
    bool InRange(double number,double num1, double num2);                 //verificar se um numero eh entre dois dados numeros
    bool IsDistanceInX(int n1, int n2);                                   //para saber a distancia entre dois nos eh no eixo x ou nao
    bool AreInTheSameAxe(int Node1,int Node2);                            //dados dois nos nos ela verifica se a distancia entre os dois nos eh no eixo x ou no y 
    bool InTheXAxe(int Node1,int Node2);                                  //dados dois nos nos parametros ela verifica se a reta entre os dois centros eh no eixo x
    bool ConditioOfNodeIsBackwards(int StartingNode, int target);         //verifica se o no desejado esta atras do robo
    bool CrossedTheBorder(int Axe);                                       //verifica se o robo ja atravessou a fronteira do proximo no
    bool RobotIsLost();                                                   //verifica se o robo esta perdido
    
    int AngleWithLeastDistance(int part);                                 //recebe o pedaco do robo e retorna a casa que tem menor distancia 
    int PartWithLeastDistance();                                          //passa em todos os pedacos e retorna o pedaco que tem a menor distancia
    int AngleToPlaceInArray(int Angle);                                   //recebe angulo e retorna a casa que pertence a ele no vetor das distancias
    int DecideFromWhereImComming();                                       //verifica o sentido que o robo esta indo
    int InWhichAxeAmI();                                                  //retorna o numero do eixo q o robo esta endando

    double DegreesToRadians(double degrees);                              //transforma graus para radianos
    double RadiansToDegrees(double radians);                              //transforma radianos para graus
    double DistanceBetweenNodes(int n1,int n2);                           //recebendo dois nos como parametro, retorna a distancia entre os dois se tiverem ligados com algum arco
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/ 

class Edges
{
    public:
    double weight;
    Node* ConnectedNode;
};

class Node
{
	public:
		double borders[4],x,y;
        int Number,Visited;
        vector<vector<int> > matrix;
		vector<Edges> adjacent;
	    void AddEdges(int vert, double distance);
	   
};

class ObstacleAvoidance
{
public:
    ObstacleAvoidance(ros::NodeHandle nh);
    void spin();
    ~ObstacleAvoidance();

//private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_, sonar_sub_, dsr_sub_;
    ros::Subscriber odom_sub_;

    nav_msgs::Odometry odom_msg_;
    geometry_msgs::Twist  desired_vel_;
    sensor_msgs::LaserScan scan_msg_;
    p2os_msgs::SonarArray sonar_msg_;


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void sonarCallback(const p2os_msgs::SonarArray::ConstPtr& sonar_msg);
    void dsrCallback(const geometry_msgs::Twist::ConstPtr& desired_vel);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg); //
    void algorithm();
};
