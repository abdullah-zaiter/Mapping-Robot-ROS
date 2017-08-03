#include "fcr2017/obstacle_avoidance.h"

/*-----------------------------------------------------------------------VARIAVEIS GLOBAIS--------------------------------------------------------------------------------*/
    double ranges[720];                                                   //vetor global das leituras do laser
    double OrganizedRanges[30];                                           //cada casa refere a 9 graus .
    double orientation[3];                                                //0 eh a coordenada no x, 1 no y e 2 eh o angulo de orientacao.
    double Target[2];                                                     //cordenadas do ponto objetivo
    double AuxStartingAngle;                                              //guarda o ultimo angulo antes de realizar a rotacao
    double DistanceError;                                                 //para corrigir o caso que o robo zere a odometria (o uso so para a realidade, na simulacao tem que continuar zero)
    
    bool callbackCalled;                                                  //para verificar se foi recebida a leitura do laser
    
    int WantedNode;                                                       //inicializei assim pois vou usar ela como flag depois, no objetivo
    int stop;                                                             //flag para fazer ele parar
    int Busy;                                                             //flag para saber se estava girando algo a n ser andar e pra continuar e terminar
    int LastNode;                                                         //ultimo no andado
    int TargetNode;                                                       //proximo no para andar
    int CurrentNode;                                                      //no atual
    int WalkedNode;                                                       //quantidade de nos andados
    int ArrivedInNode;                                                    //flag para saber se chegou no no
    int FirstTime;                                                        //flag para a verificacao do primeiro no andado para realizar a rotacao se for necessario
    int TargetBorder;                                                     //assim que entrar no outro no ele 
    int CurrentBorder;                                                    //so eh mudadada quando o robo atravessa a fronteira do no objetivo
    int WalkedNodeBorder;                                                 //contador de nos andados so que conta com a fronteira e nao com o centro do no como o "WalkedNode"
    int f;                                                                //flag para  
    
    vector<int> NodesSequence;                                            //vetor que guarda a sequencia dos nos para seguir ate o no objetivo
    
    vector<Node> graph;
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------INICIALIZAR OS OUVIDORES E FALADORES------------------------------------------------------------------------------*/
    ros::Publisher vel_pub_;
    geometry_msgs::Twist command_vel_;
    ObstacleAvoidance::ObstacleAvoidance(ros::NodeHandle nh) : nh_(nh)
    {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        laser_sub_ = nh_.subscribe("hokuyo_scan", 10, &ObstacleAvoidance::laserCallback, this);//hokuyo_scan
        dsr_sub_ = nh_.subscribe("desired_vel", 10, &ObstacleAvoidance::dsrCallback, this);
        sonar_sub_ = nh_.subscribe("sonar", 10, &ObstacleAvoidance::sonarCallback, this);
        odom_sub_ = nh_.subscribe("pose", 10, &ObstacleAvoidance::odomCallback, this);
    }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------FUNCOES DIVERSAS-----------------------------------------------------------------------------------------*/
    void HasBeenCorrupted()
    { 
        cout << "Starting Node: ";
        cin >> CurrentNode;
        CurrentBorder = CurrentNode;
        cout << "Last Node: ";
        cin >> LastNode;
        cout << "Target Node: ";
        cin >> TargetNode;
        TargetBorder = TargetNode;
        cout <<"Walked Nodes: ";
        cin >> WalkedNode;
        WalkedNodeBorder = WalkedNode-1;
        cout<< "distance error to sum !"<<endl;
        cin>>DistanceError;
        for (int i = 0; i < WalkedNode; ++i)
            graph[(NodesSequence[i])-1].Visited = 1;
        Busy = 0;
    }
    void InitializeGlobalVariables()
    {
        WantedNode = -99;                                                 
        stop=0;                                                           
        Busy = 0;
        CurrentNode = 1;                                                  
        WalkedNode = 1;                                                   
        ArrivedInNode = 0;                                                
        FirstTime = 0;                                                    
        TargetBorder = 2;                                                  
        CurrentBorder = 1;
        WalkedNodeBorder = 1;
    }

    void Delay(double time)
    {
        double t1=0,t0=0;
        t0 = ros::Time::now().toSec();
        for (t1=t0; t1-t0 < time ;)
            t1 = ros::Time::now().toSec();
    }

    bool IsDistanceInX(int n1, int n2)
    {
            double disty, distx;
            disty = abs(graph[n1-1].y -  graph[n2-1].y);
            distx = abs(graph[n1-1].x -  graph[n2-1].x);  
            if (distx>disty)
                return true;
            else
                return false;
    }

    double DistanceBetweenNodes(int n1,int n2)
    {
        double disty, distx;
        int f=1;
        
        disty = abs(graph[n1-1].y -  graph[n2-1].y);
        distx = abs(graph[n1-1].x -  graph[n2-1].x);
        if(IsDistanceInX(n1, n2))
            return distx;
        else
            return disty;
    }

    void SendVel(double linear, double angular)
    {
        command_vel_.linear.x = linear;
        command_vel_.angular.z = angular;
        vel_pub_.publish(command_vel_);
    }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------FUNCOES DE GIRAR-----------------------------------------------------------------------------------------*/
    void Turn180degrees()
    { 
        if (abs(abs(AuxStartingAngle)-180)<ANGLE_SECURITY_FACTOR_OF_ROTATING)
        {
            if (abs(orientation[2])>ANGLE_SECURITY_FACTOR_OF_ROTATING)
                SendVel(0.0,DegreesToRadians(-VEL_ANG));
            else{
                SendVel(0,0);
                Delay(0.5);
                Busy = 0;
            }
        }else if (abs(AuxStartingAngle)<ANGLE_SECURITY_FACTOR_OF_ROTATING)
        { 
            if (abs(orientation[2])<(180-(ANGLE_SECURITY_FACTOR_OF_ROTATING))){
                SendVel(0.0,DegreesToRadians(-VEL_ANG));
            }
            else{
                SendVel(0,0);
                Delay(0.5);
                Busy = 0;
            }     
        }else if (abs(AuxStartingAngle-90)<ANGLE_SECURITY_FACTOR_OF_ROTATING+5)
        { 
            if (orientation[2] > -(90-ANGLE_SECURITY_FACTOR_OF_ROTATING)){
                SendVel(0.0,DegreesToRadians(-VEL_ANG));
            }
            else{
                SendVel(0,0);
                Delay(0.5);
                Busy = 0;
            }     
        }else if (abs(abs(AuxStartingAngle)-90)<ANGLE_SECURITY_FACTOR_OF_ROTATING)
        { 
            if (orientation[2]<(90-ANGLE_SECURITY_FACTOR_OF_ROTATING)){
                SendVel(0.0,DegreesToRadians(-VEL_ANG));
            }
            else{
                SendVel(0,0);
                Delay(0.5);
                Busy = 0;
            }     
        } 
    }

    int CasesOfStartingAngle()
    {
        bool cond1, cond2, cond3, cond4;
        cond1 = (abs(AuxStartingAngle)<ANGLE_SECURITY_FACTOR_OF_ROTATING); // se ta no +X
        cond2 = (abs(AuxStartingAngle-90)<ANGLE_SECURITY_FACTOR_OF_ROTATING); // se ta no +Y
        cond3 = (abs(abs(AuxStartingAngle)-180)<ANGLE_SECURITY_FACTOR_OF_ROTATING); // se ta no -X
        cond4 = (abs(abs(AuxStartingAngle)-90)<ANGLE_SECURITY_FACTOR_OF_ROTATING); // se ta no -y

        if (cond1)
            return 1;
        else if (cond2)
            return 2;
        else if (cond3)
            return 3;
        else if (cond4)
            return 4; //testando qual e o angulo que o robo comecou para continuar e realizar a rotacao
    }


    void Turn90degrees(int IsClockwise)
    { 
        int Aux_CasesOfStartingAngle;
        if (IsClockwise==1){
            Aux_CasesOfStartingAngle = CasesOfStartingAngle();
            switch (Aux_CasesOfStartingAngle)
            {
                case 1:
                    if (orientation[2]>-(90- ANGLE_SECURITY_FACTOR_OF_ROTATING))   
                        SendVel(0,DegreesToRadians(-VEL_ANG));
                    else
                    {
                        SendVel(0,0);
                        Delay(0.5);
                        Busy = 0;
                    }
                    break;
                case 2:
                    if (orientation[2]>(ANGLE_SECURITY_FACTOR_OF_ROTATING)){   
                        SendVel(0.0,DegreesToRadians(-VEL_ANG));
                    }else
                    {
                        SendVel(0,0);
                        Delay(0.5);
                        Busy = 0;
                    }
                    break;
                case 3: 
                    if (abs(orientation[2])>(90+ANGLE_SECURITY_FACTOR_OF_ROTATING)){   
                        SendVel(0,DegreesToRadians(-VEL_ANG));
                    }else
                    {
                        SendVel(0,0);
                        Delay(0.5);
                        Busy = 0;
                    }
                    break;
                case 4:
                    if (abs(orientation[2])<(180-ANGLE_SECURITY_FACTOR_OF_ROTATING))   
                        SendVel(0,DegreesToRadians(-VEL_ANG));
                    else
                    {
                        SendVel(0,0);
                        Delay(0.5);
                        Busy = 0;
                    }
                    break;
            }
        }else if(IsClockwise==-1){
            Aux_CasesOfStartingAngle = CasesOfStartingAngle();
            switch (Aux_CasesOfStartingAngle){
                case 1:
                    if (abs(orientation[2])<(90-ANGLE_SECURITY_FACTOR_OF_ROTATING)){  
                        SendVel(0,DegreesToRadians(VEL_ANG));
                    }else
                    {
                        SendVel(0,0);
                        Delay(0.5);
                        Busy = 0;
                    }
                    break;
                case 2:
                    if (orientation[2]<(180-ANGLE_SECURITY_FACTOR_OF_ROTATING)){   //pois o angulo obitido por atan e o valor do 180 exato nao existe
                        SendVel(0,DegreesToRadians(VEL_ANG));
                    }else
                    {
                        SendVel(0,0);
                        Delay(0.5);
                        Busy = 0;
                    }
                    break;
                case 3: 
                    if (abs(orientation[2])-180>-(90-ANGLE_SECURITY_FACTOR_OF_ROTATING))   
                        SendVel(0,DegreesToRadians(VEL_ANG));
                    else
                    {
                        SendVel(0,0);
                        Delay(0.5);
                        Busy = 0;
                    }
                    break;
                case 4:
                    if (abs(orientation[2])>ANGLE_SECURITY_FACTOR_OF_ROTATING)   
                        SendVel(0,DegreesToRadians(VEL_ANG));
                    else
                    {
                        SendVel(0,0);
                        Delay(0.5);
                        Busy = 0;
                    }
                    break;
            }
        }
    }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------FUNCOES DE ANDAR E SEGUIR A TRAJETORIA-----------------------------------------------------------------------------*/
    void GoToTargetPoint()
    {   
        if (Target[0]-orientation[0]<-0.1){
            SendVel(VEL_LIN,0.0);
        }
        else if (Target[0]-orientation[0]>0.1)
        {
            if (abs(orientation[2])>5)
            {
                AuxStartingAngle = orientation[2];
                Busy = 2;
                Turn180degrees();
            }
            SendVel(VEL_LIN,0.0);   
        }
        else if (Target[1]-orientation[1]>0.1)
        {
            if (abs(orientation[2])>175)
            {
                AuxStartingAngle = orientation[2];
                Busy = 1;
                Turn90degrees(1);
            }else if (abs(orientation[2]<5))
            {
                AuxStartingAngle = orientation[2];
                Busy = -1;
                Turn90degrees(-1);   
            }
            SendVel(VEL_LIN,0.0);
        }else if (Target[1]-orientation[1]<-0.1)
        {
            if (abs(orientation[2])>175)
            {
                AuxStartingAngle = orientation[2];
                Busy = -1;
                Turn90degrees(-1);
            }else if (abs(orientation[2])<5)
            {
                AuxStartingAngle = orientation[2];
                Busy = 1;
                Turn90degrees(1);   
            }
            SendVel(VEL_LIN,0.0);
        }
        if ((abs(Target[1]-orientation[1])<0.1)&&(abs(Target[0]-orientation[0])<0.1)){
            stop = 3;
        }
    }

    void NextPoint()
    {
        while(WantedNode == -99)
        {
            cout <<"To which point you would like me to go sir ?"<<endl;
            cout <<"X: ";
            cin >> Target[0];
            cout <<"Y: ";
            cin >> Target[1];
            WhichNode(Target[0],Target[1]);
        }
        int i= NodesSequence.size();
        Path(CurrentNode, WantedNode);  
        cout <<"Wanted Node is: "<< WantedNode<<endl;
        cout<<"Sequece of nodes to follow: "<<endl;
        for (i; i < NodesSequence.size(); ++i)
            cout<<NodesSequence[i]<<" ";
        NodesSequence.erase(NodesSequence.begin() + (WalkedNode));
        TargetNode = NodesSequence[WalkedNode];
        CurrentNode = NodesSequence[WalkedNode-1];
        cout << endl;
        Busy = 0;
        stop=0;
    }

    bool AreInTheSameAxe(int Node1,int Node2)
    {
        if ((abs(graph[Node1-1].x - graph[Node2-1].x)< 0.9) || (abs(graph[Node1-1].y - graph[Node2-1].y)< 0.9))///para garantir que ele compare se os dois nos estao no mesmo eixo de coordenadas
            return true;
        else
            return false;
    }

    bool InTheXAxe(int Node1,int Node2)
    {    
        if ((abs(graph[Node1-1].y - graph[Node2-1].y) < 0.2))
            return true;
        else
            return false;
    }

    int InWhichAxeAmI()
    {
        if (InTheXAxe(CurrentNode,TargetNode))
        {
            if (graph[CurrentNode-1].x<graph[TargetNode-1].x) return 0;
            else return 1;
        }else
        {
            if (graph[CurrentNode-1].y<graph[TargetNode-1].y) return 2;
            else return 3;
        }            
    }

    bool RobotIsLost()
    {  
        switch(InWhichAxeAmI())
        {
            case 0:
                if(abs(orientation[2])<CORRECTION_ANGLE_FACTOR) return false;           //se o angulo esta fora da margem de erro permitida para esse eixo
                else return true;
                break;
            case 1:
                if(abs(abs(orientation[2])-180)<CORRECTION_ANGLE_FACTOR) return false;
                else return true;
                break;
            case 2:
                if(abs(orientation[2]-90)<CORRECTION_ANGLE_FACTOR) return false;
                else return true;
                break;
            case 3:
                if(abs(abs(orientation[2])-90)<CORRECTION_ANGLE_FACTOR) return false;
                else return true; 
                break; 
        }
    }

    void FixOrientation(int Axe)
    {
        switch(Axe)
        {
            case 0:
                if(AuxStartingAngle<0){
                    if (orientation[2]<-ANGLE_SECURITY_FACTOR_OF_ROTATING){
                        SendVel(0.0,DegreesToRadians(VEL_ANG));
                    }
                    else
                    {
                        SendVel(0.0,0.0);
                        Busy = 0;
                    }
                }else
                {
                    if (orientation[2]>ANGLE_SECURITY_FACTOR_OF_ROTATING)
                        SendVel(0.0,DegreesToRadians(-VEL_ANG));
                    else
                    {
                        SendVel(0.0,0.0);
                        Busy = 0;
                    }   
                }
                break;
            case 1:
                if(AuxStartingAngle<0){
                    if (abs(abs(orientation[2])-180)>ANGLE_SECURITY_FACTOR_OF_ROTATING)
                        SendVel(0.0,DegreesToRadians(-VEL_ANG));
                    else
                    {
                        SendVel(0.0,0.0);
                        Busy = 0;
                    }
                }else   
                {
                    if (abs(orientation[2]-180)>ANGLE_SECURITY_FACTOR_OF_ROTATING)
                        SendVel(0.0,DegreesToRadians(VEL_ANG));
                    else
                    {
                        SendVel(0.0,0.0);
                        Busy = 0;
                    }  
                }
                break;
            case 2:
                if(abs(AuxStartingAngle)>90){
                    if ((abs(orientation[2])-90)>ANGLE_SECURITY_FACTOR_OF_ROTATING)
                        SendVel(0.0,DegreesToRadians(-VEL_ANG));
                    else
                    {
                        SendVel(0.0,0.0);
                        Busy = 0;
                    }
                }else   
                {
                    if (abs(orientation[2]-90)>ANGLE_SECURITY_FACTOR_OF_ROTATING)
                        SendVel(0.0,DegreesToRadians(VEL_ANG));
                    else
                    {
                        SendVel(0.0,0.0);
                        Busy = 0;
                    }  
                }
                break;
            case 3:
                if(abs(AuxStartingAngle)>90){
                    if (abs(abs(orientation[2])-90)>ANGLE_SECURITY_FACTOR_OF_ROTATING)
                        SendVel(0.0,DegreesToRadians(VEL_ANG));
                    else
                    {
                        SendVel(0.0,0.0);
                        Busy = 0;
                    }
                }else   
                {
                    if (abs(abs(orientation[2])-90)>ANGLE_SECURITY_FACTOR_OF_ROTATING)
                        SendVel(0.0,DegreesToRadians(-VEL_ANG));
                    else
                    {
                        SendVel(0.0,0.0);
                        Busy = 0;
                    }  
                }
                break;
        }
    }
    bool IsLogicalOrientationError()
    {
        switch (DecideFromWhereImComming())
        {
            case 0:
                if (abs(orientation[2])>40)
                    return false;
                else
                    return true;
                break;
            case 1:
                if (abs(abs(orientation[2])-180)>40)
                    return false;
                else
                    return true;
                break;
            case 2:
                if (abs(orientation[2]-90)>40)
                    return false;
                else
                    return true;
                break;
            case 3:
                if (abs(abs(orientation[2])-90)>40)
                    return false;
                else
                    return true;
                break;
        }
    }
    void WalkFromNodeToAnother(int StartingNode, int Target)
    {    
       if (!RobotIsLost())
        {
            if (InTheXAxe(StartingNode,Target))
            {
                if (abs(orientation[0]-graph[Target-1].x)>0.05) ///para garantir que ele chegou pro centro do proximo no
                {
                    SendVel(VEL_LIN,0.0);
                    DealingWithFile();
                }else{
                    SendVel(0.0,0.0);
                    ArrivedInNode=1;
                    LastNode = StartingNode;
                }          
            }else
            {
                if (abs(orientation[1]-graph[Target-1].y)>0.05) ///para garantir que ele chegou pro centro do proximo no
                {
                    SendVel(VEL_LIN,0.0);
                    DealingWithFile();
                }else
                {
                    SendVel(0.0,0.0);
                    ArrivedInNode=1;
                    LastNode = StartingNode;
                }
            }
        }else{
            if(IsLogicalOrientationError())
            {
                Busy = 6;
                AuxStartingAngle = orientation[2];
                SendVel(0.0,0.0);
                FixOrientation(InWhichAxeAmI());    
            }else
                SendVel(VEL_LIN,0.0);
        }
    }

    bool ConditioOfNodeIsBackwards(int StartingNode, int Target)
    {                                            
        bool cond1,cond2,cond3,cond4;

        cond1 = ((graph[StartingNode-1].x > graph[Target-1].x) && (abs(orientation[2])<ANGLE_SECURITY_FACTOR_OF_ROTATING));         //se ta apontando pra direita no x e o no esta atras             
        cond2 = ((graph[StartingNode-1].x < graph[Target-1].x) && (abs(abs(orientation[2])-180)<ANGLE_SECURITY_FACTOR_OF_ROTATING));//se ta apontando pra esquerda no x e o no esta atras   
        cond3 = ((graph[StartingNode-1].y > graph[Target-1].y) && ((orientation[2]-90)<ANGLE_SECURITY_FACTOR_OF_ROTATING));         //se ta apontando para cima e o no ta atras        
        cond4 = ((graph[StartingNode-1].y < graph[Target-1].y) && (abs(orientation[2]+90)<ANGLE_SECURITY_FACTOR_OF_ROTATING));      //se ta apontando pra baixo e no ta atras   
        
        if (cond1||cond2||cond3||cond4){
            return true;
        }
        else
            return false;
    }


    void TurnTheRobotIfItNeedsIt(int StartingNode, int Target)
    {   
        cout<<"last current target "<<LastNode<<" "<<CurrentNode<<" "<<TargetNode<<endl;
        if (!AreInTheSameAxe(LastNode, Target))
        {
            if (InTheXAxe(StartingNode,Target))
            {   
                if (graph[StartingNode-1].x < graph[Target-1].x ){
                    AuxStartingAngle = orientation[2];
                    if(CasesOfStartingAngle()==4){
                        Busy = -1;
                        Turn90degrees(-1);
                    }else{
                        Busy = 1;
                        Turn90degrees(1);
                    }
                }else
                {
                    AuxStartingAngle = orientation[2];
                    if(CasesOfStartingAngle()==4){
                        Busy = 1;
                        Turn90degrees(1);
                    }else{
                        Busy = -1;
                        Turn90degrees(-1);
                    }
                }
            }else
            {
                if (graph[StartingNode-1].y < graph[Target-1].y){
                    AuxStartingAngle = orientation[2];
                    if(CasesOfStartingAngle()==3){
                        Busy = 1;
                        Turn90degrees(1);
                    }else{
                        Busy = -1;
                        Turn90degrees(-1);
                    }
                    
                }else
                {
                    AuxStartingAngle = orientation[2];
                    if(CasesOfStartingAngle()==3){
                        Busy = -1;
                        Turn90degrees(-1);
                    }else{
                        Busy = 1;
                        Turn90degrees(1);
                    }
                }
            }
        }
        else if (ConditioOfNodeIsBackwards(StartingNode,Target)) //se o no de objetivo esta atras do no atual que o robo esta
        {
            AuxStartingAngle = orientation[2];
            Busy = 2;
            Turn180degrees();
        }
    }

    void SequenceOfNodesToWalk()
    {
        if (WalkedNode<NodesSequence.size())
        {
            if (FirstTime==0){
                TurnTheRobotIfItNeedsIt(CurrentNode,TargetNode);
                FirstTime+=1;
            }
            if (ArrivedInNode)
            {            
                CurrentNode = TargetNode;       
                WalkedNode++;
                if (WalkedNode<NodesSequence.size())
                {
                    TargetNode = NodesSequence[WalkedNode];
                }
                cout <<"Im in the center!\n"<<"Current: "<<CurrentNode<<" Target: "<<TargetNode<<" Last Node: "<<LastNode<<endl;     
                ArrivedInNode = 0;
                Delay(1.0); 
                TurnTheRobotIfItNeedsIt(CurrentNode,TargetNode);
                WalkFromNodeToAnother(CurrentNode,TargetNode); 
            }else{
                WalkFromNodeToAnother(CurrentNode,TargetNode);
            }
        }else if (NodesSequence.size()<11)
            stop=1;
        else{
            Busy = 5;
            stop = 2;
        }
    }

    void GoToTheCenter()
    {
        if (abs(orientation[0])<abs(graph[0].x))
            SendVel(-VEL_LIN,0.0);
        else{
            SendVel(0.0,0.0);
            PrepareTheMatrix(CurrentBorder);
            Busy = 0;
        }
    }

    void AnalyseTheSituation() // analisa o caso dos flags antes de fazer algum ato
    {
        switch(Busy)
        {
            case 1:
                Turn90degrees(Busy);
                break;
            case -1:
                Turn90degrees(Busy);
                break;
            case 2:
                Turn180degrees();
                break;
            case 3:
                //if (!ThereIsAnObstacle())
                    GoToTheCenter();
                break;
            case 4:
                cout<<"Avoiding"<<endl;
                AvoidTheObstacle();
                break;
            case 5:
                if (!ThereIsAnObstacle())
                    GoToTargetPoint();
                break;
            case 6:
                FixOrientation(InWhichAxeAmI());
                break;
            case 0:
                if (!ThereIsAnObstacle())
                    SequenceOfNodesToWalk();
                break;
        }
    }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------FUNCOES DO MAPEAMENTO--------------------------------------------------------------------------------------*/
    int DecideFromWhereImComming()
    {
        int flag;
        if (graph[CurrentBorder-1].x - graph[TargetBorder-1].x < -2)      //vindo de -x para +x
            flag = 0; 
        else if (graph[CurrentBorder-1].x - graph[TargetBorder-1].x > 2)  //vindo de +x para -x
            flag = 1;
        else if (graph[CurrentBorder-1].y - graph[TargetBorder-1].y < -2) //vindo de -y para +y
            flag = 2;
        else if (graph[CurrentBorder-1].y - graph[TargetBorder-1].y > 2)  //vindo de +y para -y
            flag = 3;
        return flag;
    }

    bool CrossedTheBorder(int Axe)
    {
        if (Axe == 1)
        {
            if (abs(orientation[0]-graph[TargetBorder-1].borders[DecideFromWhereImComming()])<0.05) return true;
            else return false;
        }else if (Axe == 2)
        {
            if (abs(orientation[1]-graph[TargetBorder-1].borders[DecideFromWhereImComming()])<0.05) return true;
            else return false;           
        }
    }

    void DealingWithFile()
    {    
        //cout<<"heyhey"<<endl;
        if(DecideFromWhereImComming() == 0 || DecideFromWhereImComming() == 1)
        {
            if (CrossedTheBorder(1))
            {
                WalkedNodeBorder+=1;
                if (graph[CurrentBorder-1].Visited == 0)    
                    CreateAndFillAFile(CurrentBorder);
                graph[CurrentBorder-1].Visited = 1;
                CurrentBorder = TargetBorder;
                if (WalkedNodeBorder<NodesSequence.size())
                    TargetBorder = NodesSequence[WalkedNodeBorder];
                if (graph[CurrentBorder-1].Visited == 0)    
                    PrepareTheMatrix(CurrentBorder);    
            }else
            {  
                if (graph[CurrentBorder-1].Visited == 0){
                    MakeTheGrid(CurrentBorder);
                    CreateAndFillAFile(CurrentBorder);
                }
            }
        }else if(DecideFromWhereImComming() == 2 || DecideFromWhereImComming() == 3)
        {
            //cout<<"yowyow"<<endl;
            if(CrossedTheBorder(2))
            {
                WalkedNodeBorder+=1;
                if (graph[CurrentBorder-1].Visited == 0)    
                    CreateAndFillAFile(CurrentBorder);
                graph[CurrentBorder-1].Visited = 1;
                CurrentBorder = TargetBorder;
                if (WalkedNodeBorder<NodesSequence.size())
                    TargetBorder = NodesSequence[WalkedNodeBorder];
                if (graph[CurrentBorder-1].Visited == 0)    
                    PrepareTheMatrix(CurrentBorder);    
            }else
            {  
                if (graph[CurrentBorder-1].Visited == 0){
                    //cout<<"oi to aquiii"<<endl;
                    MakeTheGrid(CurrentBorder);
                    CreateAndFillAFile(CurrentBorder);
                }

            }
        }
    }

    void PrepareTheMatrix(int NodeNumber)
    {
        double DeltaX,DeltaY;
        DeltaX = graph[NodeNumber-1].borders[X2] - graph[NodeNumber-1].borders[X1];
        DeltaY = graph[NodeNumber-1].borders[Y2] - graph[NodeNumber-1].borders[Y1];
        int i,j;
        i = round(DeltaY*4);//2);
        j = round(DeltaX*4);//2);
        graph[NodeNumber-1].matrix.resize(i, vector<int>(j));
        memset( &graph[NodeNumber-1].matrix[0][0], 0 , sizeof(graph[NodeNumber-1].matrix));
    }

    void CreateAndFillAFile(int NodeNumber)
    {
        
        char FileName[17] = {'G','r','i','d','s','/','N','o','d','e',' ',' ','.','t','x','t','\0'};
        char aux;
        aux = '0'+((NodeNumber-(NodeNumber%10))/10);
        FileName[10] = aux;
        aux = '0'+(NodeNumber%10);
        FileName[11] = aux;
        ofstream outputFile(FileName);
        for (int i = (graph[NodeNumber-1].matrix.size()-1); i >= 0 ; --i)
        {
            for (int j = 0; j < graph[NodeNumber-1].matrix[0].size(); ++j)
                outputFile << graph[NodeNumber-1].matrix[i][j]<<" ";
            outputFile <<endl;
        }
        outputFile <<endl<<endl;
        outputFile.close();
    }

    void MakeTheGrid(int NodeNumber)
    {
        int placeinvector;
        double dx, xmat, xrec, dy, ymat, yrec, theta, rs, rt;
        for (int i = 0; i < graph[NodeNumber-1].matrix.size(); ++i)
        {
            
            for (int j = 0; j < graph[NodeNumber-1].matrix[0].size(); ++j)
            {
                if (DecideFromWhereImComming()==0 ||DecideFromWhereImComming()==1)
                {
                    dx = abs(orientation[0]- graph[NodeNumber-1].borders[DecideFromWhereImComming()]);
                    dy = abs(orientation[1]- graph[NodeNumber-1].borders[Y1]);
                }else if (DecideFromWhereImComming()==2 ||DecideFromWhereImComming()==3)
                {
                    dx = abs(orientation[0]- graph[NodeNumber-1].borders[X1]);
                    dy = abs(orientation[1]- graph[NodeNumber-1].borders[DecideFromWhereImComming()]);
                }       
                xmat = j/4 + 0.25; //- 0.25;//j/2 + 0.5;
                xrec = xmat - dx;
                ymat = i/4 + 0.25; //- 0.25;//i/2 + 0.5;
                yrec = ymat - dy;
                RectangularToPolar(xrec, yrec, &rt, &theta);
                //cout<<"rt: "<<rt<<" rs: "<<rs<<" test222"<<endl;
                if (DecideFromWhereImComming()==2 ||DecideFromWhereImComming()==3)
                {
                    if(theta>-45)
                        theta += 45; 
                    else
                        theta += 360+45;
                }else
                    theta += 135;
                
                if (theta<=255 && theta>=15)
                {    
                    placeinvector = theta/9;
                    rs = OrganizedRanges[placeinvector];
                    //cout<<"rt: "<<rt<<" rs: "<<rs<<" test1"<<endl;
                    if (rt - rs > 0.1 && rt - rs < 0.5)
                        graph[NodeNumber-1].matrix[i][j] = 2;        
                    else if (rt<rs)
                        graph[NodeNumber-1].matrix[i][j] = 1;
                }        
            }
        } 
    }

    void PrintGrade(int NodeNumber)
    {
        
        for (int i = 0; i < graph[NodeNumber-1].matrix.size(); ++i)
        {
            for (int j = 0; j < graph[NodeNumber-1].matrix[0].size(); ++j)
            {
                cout<<graph[NodeNumber-1].matrix[i][j]<<" ";
            }
            cout<<endl;
        }
        cout<<endl<<endl;
    }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------FUNCOES DO GRAFO-----------------------------------------------------------------------------------------*/
    void Path(int CurrentNode, int WantedNode)
    {
        double Other_Ways,Min; /// caminhos alternativos ao no atual
        int k;

        vector<Node*> Not_visited_yet; //vetor de ponteiros nao visitados ainda
        Node* u = NULL;
        Node* v = NULL; //dois ponteiros pra nos que facilitam a escrita do codigo
        
        vector<Node*> Nodes_sequence(graph.size(),NULL); /// aponta nos nos conectados com um dado no, no caminho
        vector<double> Distance(graph.size(),999999999); // guarda as distancias ate o no inicial, comecado com distancia mto grande

        for (int i = 0; i < graph.size(); ++i)
        {
            Not_visited_yet.push_back(&graph[i]);   
        } /// passando os enderecos dos nos do grafo pro vetor de ponteiros n visitados ()

        Distance[CurrentNode-1] = 0;
            ///distancia do no pro proprio no e zero

        while(Not_visited_yet.size()>0 && u != &graph[WantedNode-1])
        {
            v = Not_visited_yet[0];
            Min = Distance[v->Number-1];
            u = v;
            k=0;
            for (int i = 0; i < Not_visited_yet.size(); i++)
            {
                v = Not_visited_yet[i];
                if (Distance[v->Number-1]<Min)
                {
                    Min = Distance[v->Number-1];
                    u = v;
                    k= i;
                }
            }
            Not_visited_yet.erase(Not_visited_yet.begin()+k);

            for (int i = 0; i < u->adjacent.size(); ++i)
            {
                v = u->adjacent[i].ConnectedNode;
                Other_Ways = Distance[u->Number-1] + u->adjacent[i].weight;
                if (Other_Ways < Distance[v->Number-1])
                {
                    Distance[v->Number-1] = Other_Ways;
                    Nodes_sequence[v->Number-1] = u;
                }

            }
        }

        vector<Node> LastSequence;
        while(Nodes_sequence[u->Number-1] != NULL)
        {
            LastSequence.insert(LastSequence.begin(),*u);
            u = Nodes_sequence[u->Number-1]; 
        }

        LastSequence.insert(LastSequence.begin(),*u);
        for (int i = 0; i < LastSequence.size(); ++i)
            NodesSequence.push_back(LastSequence[i].Number);
    }

    void PrintGraph()//funcao para imprimir o grafo
    {     
        for (int i = 0; i<graph.size(); ++i)
        {
            cout << "no: "<<graph[i].Number<< endl <<" x postition "<<graph[i].x<<"  y postition "<<graph[i].y <<endl;///numero do no = i+1, seria numerado pelas casas no vetor
            cout << "  borders X1, X2, Y1, Y2: ";
            for (int j = 0; j < 4; ++j)
                cout<<graph[i].borders[j]<<", ";
            cout<< endl << "\tadjacents ";
            for(int j=0 ; j < graph[i].adjacent.size() ; j++)
                cout << graph[i].adjacent[j].weight << " ";

            cout<< endl;
        }
        cout<<" __________________________"<<endl;
        cout<<"|13|_12_|11|_10_|09|_08_|07|"<<endl;
        cout<<"|14|____|17|____|18|____|06|"<<endl;
        cout<<"|15|_16_|01|_02_|03|_04_|05|"<<endl;
    }

    bool InRange(double number,double num1, double num2)//funcao para testar se um numero e entre dois
    {
        if ((num1<number)&&(num2>number))
            return true;
        else
            return false;
    }

    
    void Node::AddEdges(int vert, double distance)
    {
            Edges temp;
            temp.weight = distance;
            temp.ConnectedNode = &graph[vert-1];
            adjacent.push_back(temp);
    }
    bool CreatGraph()//funcao para criar o grafo
    {   
        int i=0,nodes,verts,j=0,aux_vert=0;
        double distance;
        ifstream GraphTxt ("graph.txt"); // abrir arquivo (o programa tem que ser executado na mesma pasta q o arquivo que tem os dados do grafo esta).
        GraphTxt >> nodes;
        graph.resize(nodes);
        if (GraphTxt.is_open())
        {  
            cout <<nodes<<" nodes, graph file is opened !\n";
            for (int i = 0; (i<nodes) ; ++i)
            {              
                Node temp;
                GraphTxt >> temp.borders[X1];
                GraphTxt >> temp.borders[X2];
                GraphTxt >> temp.borders[Y1];
                GraphTxt >> temp.borders[Y2];
                temp.x = (((temp.borders[X1] - temp.borders[X2]) / 2.0 ) + temp.borders[X2] ); // calculo para obter as coordenadas x e y do centro do apartir das fronteiras pegas do arquivo texto
                temp.y = (((temp.borders[Y1] - temp.borders[Y2]) / 2.0 ) + temp.borders[Y2] );
                GraphTxt >> temp.Number;
                temp.Visited = 0;
                GraphTxt >> verts;
                for (int j = 0; (j<verts); ++j)
                {
                    GraphTxt >> aux_vert;
                    GraphTxt >> distance;
                    temp.AddEdges(aux_vert,distance);
                }
                graph[i]=temp;
            }
            return true;       
        }else{
            cout << "************************************************************************\n"; 
            cout << "Graph file isn't opened!\nAre you sure that you are in the right directory to execute the program?\nMake sure to be in /catkin_ws/src/fcr2017/src\n";
            cout << "************************************************************************\n";
            return false;
        }
    }

    void WhichNode(double x, double y)//funcao para saber a qual no pertencem as coordenadas passadas em parametro
    {   
        for (int i = 0; ((i < graph.size())&&(WantedNode==-99)); i++)
            if (InRange(x,graph[i].borders[X1],graph[i].borders[X2]) && InRange(y,graph[i].borders[Y1],graph[i].borders[Y2])) //se ta dentro as fronteiras de x e de y do no
                WantedNode = i+1; 
    }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------FUNCOES DE COVERSAO----------------------------------------------------------------------------------------*/
    void RectangularToPolar(double x, double y,double *r, double *theta)
    {
        *r = sqrt(pow(x,2)+pow(y,2));
        *theta = RadiansToDegrees(atan2(y,x));
    }
    
    void PolarToRectangular(double r, double theta,double *x, double *y)
    {
        *x = (r * cos(DegreesToRadians(theta)));
        *y = (r * sin(DegreesToRadians(theta)));
    }

    int AngleToPlaceInArray(int Angle)
    {
        int place;
        Angle += 135;
        place = round(Angle/(24*0.375));
        return place;
    }
    
    double DegreesToRadians(double degrees)
    {        
        return degrees*PI/180;
    }

    double RadiansToDegrees(double radians)
    {
        return radians*180/PI;
    }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------FUNCOES DE DESVIAR DE OBSTACULOS----------------------------------------------------------------------------------*/
    
    /*bool IsNodeInY(int Node)
    {
        if (Node == 14 ||Node == 17 ||Node == 18 ||Node == 6)
            return true;
        else
            return false;
    }*/

    bool RightSide()
    {
        double avgl=0,avgr=0;
        //cout << "oi to aquiiiii"<<endl;
        for (int i = 0; i < 15; ++i)
            avgr += OrganizedRanges[i];
        for (int i = 15; i < 30; ++i)
            avgl += OrganizedRanges[i];
        avgr /= 15;
        avgl /= 15;
        if (avgr>avgl)
            return true;
        else
            return false;
    }

    int PartWithLeastDistance() // saber qual pedaco mesmo tem a menor distancia do obstaculo
    {
        int i=0, LeastPart=0;
        for (int i = 0; i < 30; ++i)
            if(OrganizedRanges[i]< OrganizedRanges[LeastPart])
                LeastPart = i;
        return LeastPart;
    }

    bool ThereIsAnObstacle2()
    {
        int flag=0;
        for (int i = ((135-(AVOIDING_ANGLE/2))/9) ; i < ((135+(AVOIDING_ANGLE/2))/9)  ; i++ ) //dividido por dois pois o comeca da metade do angulo no lado direito e vai ate a metade no lado esquerdo... a multiplicacao pois ele conta em meio angulo..... +10 para pegar a casa do meio no pedaco analisado
            if (OrganizedRanges[i]<MINIMUN_DISTANCE)
                flag++;
        if (flag!=0)
            return true;
        else
            return false;
    }

    bool ThereIsAnObstacle()
    {
        int flag=0;
        for (int i = ((135-(AVOIDING_ANGLE/2))/9) ; i < ((135+(AVOIDING_ANGLE/2))/9)  ; i++ ) //dividido por dois pois o comeca da metade do angulo no lado direito e vai ate a metade no lado esquerdo... a multiplicacao pois ele conta em meio angulo..... +10 para pegar a casa do meio no pedaco analisado
        {
            if (OrganizedRanges[i]<MINIMUN_DISTANCE)
                flag++;
        }
        if (flag!=0){
            Busy = 4;
            return true;
        }
        else
            return false;
    }

    void PrintTheSensorArray() //imprimir o vetor do sensore recebe como parametro o passo entre cada angulo/2 impresso e o passado ----- para realizar testes -----
    {
        for(int i = 0; i < 30 ; i++)
            cout<<"Distance :  " << OrganizedRanges[i] << "  Angle : " << (i)*9 << endl; 
        cout << endl << endl; 
    }

    int AngleWithLeastDistance(int part) /// achar qual parte do pedaco tem a menor distancia
    {
        int LeastAngle = part*24, aux, i = part*24;
        while(i < (part*24+24))
        {   
            if (ranges[i]<ranges[LeastAngle])
               LeastAngle = i;
            i++;
        }
        return LeastAngle;
    }

    void FuseTheAngles(int Angle, int part) //guardar o valor no vetor q cada casa dele refere a 9 graus.
    {
        OrganizedRanges[part] = ranges[Angle];
    }

    void ArrayDivider()
    {
        int Angle;
        for (int i = 0; i < 30; ++i) // dividir para 18 pedacos
        {
            Angle = AngleWithLeastDistance(i); // pra cada pedaco saber qual e o angulo que tem a menor distancia
            FuseTheAngles(Angle,i); // pra cada pedaco faz todas as casas do vetor com distancias iguais a menor distancia que tem no pedaco
        }
    }
    void AvoidTheObstacle()
    {
        if (ThereIsAnObstacle2()){
            SendVel(0,0);
            switch (DecideFromWhereImComming())
            {
                case 0:
                    if (orientation[1]<graph[CurrentBorder-1].y)
                    {
                        AuxStartingAngle = orientation[2];
                        Busy = -1;
                        Turn90degrees(-1);
                    }else
                    {
                        AuxStartingAngle = orientation[2];
                        Busy = 1;
                        Turn90degrees(1);
                    }
                    break;
                case 1:
                    if (orientation[1]<graph[CurrentBorder-1].y)
                    {
                        AuxStartingAngle = orientation[2];
                        Busy = 1;
                        Turn90degrees(1);
                    }else
                    {
                        AuxStartingAngle = orientation[2];
                        Busy = -1;
                        Turn90degrees(-1);
                    }         
                    break;
                case 2:
                    if (orientation[0]<graph[CurrentBorder-1].x)
                    {
                        AuxStartingAngle = orientation[2];
                        Busy = 1;
                        Turn90degrees(1);

                    }else
                    {
                        AuxStartingAngle = orientation[2];
                        Busy = -1;
                        Turn90degrees(-1);
                    } 
                    break;
                case 3:
                    if (orientation[0]<graph[CurrentBorder-1].x)
                    {
                        AuxStartingAngle = orientation[2];
                        Busy = -1;
                        Turn90degrees(-1);
                    }else
                    {
                        AuxStartingAngle = orientation[2];
                        Busy = 1;
                        Turn90degrees(1);
                    } 
                    break;
            }
        }else
        { 
            if (stop == 0 || stop == 1){
                Busy = 0;
                SendVel(VEL_LIN,0.0);
            }else if (stop == 2){
                Busy = 5;
                SendVel(VEL_LIN,0.0);
            }
        }
    }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------FUNCOES DE PEGAR A LEITURA DOS SENSORES------------------------------------------------------------------------------*/

    void ObstacleAvoidance::laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
    {
        this->scan_msg_ = *laser_msg;
        for (int i = 0; i < 720; ++i)
            ranges[i] = scan_msg_.ranges[i];
        callbackCalled = true;
    }


    void ObstacleAvoidance::dsrCallback(const geometry_msgs::Twist::ConstPtr& desired_vel)
    {
        
        this->desired_vel_ = *desired_vel;
    }

    void ObstacleAvoidance::sonarCallback(const p2os_msgs::SonarArray::ConstPtr& sonar_msg)
    {
        
        this->sonar_msg_ = *sonar_msg;
    }

    void ObstacleAvoidance::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        
        this->odom_msg_ = *odom_msg;
    }
/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void ObstacleAvoidance::algorithm()
{
    if (f)
        orientation[0] = odom_msg_.pose.pose.position.x+DistanceError;     
    else
        orientation[0] = odom_msg_.pose.pose.position.x;
    orientation[2] = RadiansToDegrees(tf::getYaw(odom_msg_.pose.pose.orientation));
    orientation[1] = odom_msg_.pose.pose.position.y;
    //cout<<"busy= "<<Busy<<" Current: "<<CurrentNode<<" TargetNode: "<<TargetNode<<" Walked node: "<<WalkedNode<<" WalkedBorder: "<<WalkedNodeBorder<<endl;
    //cout<<"orientation: "<<orientation[2]<<endl;
    if (stop==1){    
        SendVel(0.0,0.0);
        cout<<"Waiting for next command"<<endl;
        Delay(0.5);
        NextPoint();
    }
    if (stop == 3)
    {
        SendVel(0.0,0.0);
        cout<<"Reached point ("<<Target[0]<<","<<Target[1]<<")"<<endl;
        cout<<"And its done, verify the created files of the map, Thanks !"<<endl;
        exit(0);
    }
    ArrayDivider();
    AnalyseTheSituation();
}

void ObstacleAvoidance::spin()
{
    ros::Rate loop_rate(50);
    if(CreatGraph())
        PrintGraph();
    else
        exit(0);
    f=0;
    InitializeGlobalVariables();
    NodesSequence.resize(10);
    NodesSequence[0] = 1; 
    NodesSequence[1] = 2;
    NodesSequence[2] = 3;
    NodesSequence[3] = 4;
    NodesSequence[4] = 5;
    NodesSequence[5] = 4;
    NodesSequence[6] = 3;
    NodesSequence[7] = 18;
    NodesSequence[8] = 9;
    NodesSequence[9] = 18;
    LastNode = 16;
    cout<<"Are you starting the robot for the first time ?\n1 for no, 0 for yes"<<endl;
    cin>>f;
    DistanceError=0;
    if(f)
        HasBeenCorrupted();
    else
    {
        CurrentNode = NodesSequence[0];
        TargetNode = NodesSequence[1];   
    }

    cout<<"Sequece of node to follow: "<<endl;
    for (int i = 0; i < NodesSequence.size(); ++i)
        cout<<NodesSequence[i]<<" ";
    cout << endl;
       
    while(ros::ok())
    {
        callbackCalled = false;
        ros::spinOnce();
        //if (callbackCalled == true)
        algorithm();
        //else
        //    SendVel(0.0,0.0); 
            
        loop_rate.sleep();
    }
}

ObstacleAvoidance::~ObstacleAvoidance()
{
}