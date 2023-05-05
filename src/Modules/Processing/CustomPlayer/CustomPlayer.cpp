#include "CustomPlayer.h"

CustomPlayer::CustomPlayer(int index, QThreadPool* threadPool) : Processing(index, threadPool) {
}

void CustomPlayer::buildParameters(Parameters::Handler& parameters) {
}

void CustomPlayer::connectModules(const Modules* modules) {
  connect(modules->vision(),
          &Vision::sendFrame,
          this,
          &CustomPlayer::receiveFrame,
          Qt::DirectConnection);

  connect(modules->vision(),
          &Vision::sendField,
          this,
          &CustomPlayer::receiveField,
          Qt::DirectConnection);
}

void CustomPlayer::init(const Modules* modules) {
}

void CustomPlayer::update() {
  shared->field.extract_to(field);
  if (auto f = shared->frame.get_optional_and_reset()) {
    if (auto it = f->allies().findById(index()); it != f->allies().end()) {
      robot = *it;
    }
    frame.emplace(*f);
  }
}

double min(double a, double b) //auto explicativa.
{
  if(a <= b)
  {
    return a;
  }
  else
  {
    return b;
  }
}

double max(double a, double b) //auto explicativa.
{
  if(a >= b)
  {
    return a;
  }
  else
  {
    return b;
  }
}


Point calculaPasso (int ID, Point point, float passo, 
                      std::optional<Field> field, std::optional<Frame> frame)
//Calcula em qual direcao deve ser o proximo passo do robo baseado em sua posicao atual.
{
  /*
  O robo pode seguir 8 direcoes

   5  6  7
  3  ROBO  4
   0  1  2

  */

  //Primeira etapa: atribuir um ponto a cada uma das 8 direcoes possiveis.
  //Cada um destes pontos ficara a uma distancia de 'passo' do robo.
  Point caminhos[8] = {point, point, point, point, 
                      point, point, point, point};

  caminhos[0][0] -= 0.7 * passo; caminhos[0][1] -= 0.7 * passo;
  caminhos[1][1] -= passo;
  caminhos[2][0] += passo; caminhos[2][1] -= 0.7 * passo;
  caminhos[3][0] -= passo;
  caminhos[4][0] += passo;
  caminhos[5][0] -= 0.7 * passo; caminhos[5][1] += 0.7 * passo;
  caminhos[6][1] += passo;
  caminhos[7][0] += 0.7 * passo; caminhos[7][1] += 0.7 * passo;

  double pesos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  //array que vai armazenar as prioridades de cada um dos 8 caminhos.
  //O peso inicial eh 0 para todos.

  int ponto_mais_viavel = 0; //guarda o indice do caminho de maior peso.

  float lim_esquerda = field->topLeft().x();
  float lim_direita = field->topRight().x();
  float lim_superior = field->topLeft().y();
  float lim_inferior = field->bottomLeft().y();

  for(int i = 0; i < 8; i++)
  //for que percorre todos os caminhos possiveis atribuindo pesos a cada um.
  {


    if(caminhos[i][0] >= lim_esquerda && caminhos[i][0] <= lim_direita
        && caminhos[i][1] >= lim_inferior && caminhos[i][1] <= lim_superior)
    // Primeira checagem: o ponto em questao estah contido no campo? Se sim, serah atribuido
    //um peso diferente de 0. Se não, o ponto vai ficar sem peso por apontar para fora do campo.
    {
      
      if(frame->ball().distTo(caminhos[i]) > passo/10)
      //Este 'if' serve apenas para desencargo, para evitar divisao por um numero muito pequeno.
      {
        pesos[i] = (10/(frame->ball().distTo(caminhos[i])));
        //Segunda checagem: o ponto receberah um peso inversamente proporcional a sua distancia
        //pra bola, portanto, pontos mais proximos da bola serao priorizados.
      }

      for(int j = 0; j < 3; j++)
      //Terceira checagem: a distancia do ponto pra cada robo, aliado ou inimigo.
      {
        if(j != ID && frame->allies().findById(j)->distTo(caminhos[i]) < 15.0)
        //Se houver um aliado proximo do ponto, reduz o peso (queremos desviar do obstaculo).
        {
          pesos[i] *= (frame->allies().findById(j)->distTo(caminhos[i]) / 15);
        }

        if(frame->enemies().findById(j)->distTo(caminhos[i]) < 15.0)
        //Se houver um inimigo proximo do ponto, reduz o peso (queremos desviar do obstaculo).
        {
          pesos[i] *= (frame->enemies().findById(j)->distTo(caminhos[i]) / 15);
        }
        
      }
      if((frame->allies().findById(ID)->x() - frame->ball().x()) 
          * ((int) field->isAttackingToRight() - 1) > 0 && 
          frame->ball().distTo(caminhos[i]) < 15)
      //Ultima checagem: (acabou nao fazendo muita diferenca ;-; )
      //Checa se o robo em questao esta na frente da bola. Como preferimos que o robo sempre
      // se aproxime da bola por tras (para evitar gol contra), queremos que ele trate a
      // propria bola como obstaculo se ele se aproximar pela frente, portanto pontos proximos
      // a bola terao seu peso reduzido.
      {
        pesos[i] *= std::pow(frame->ball().distTo(caminhos[i])/15, 4);
      }

    }
     if(pesos[ponto_mais_viavel] < pesos[i]) //Compara com o maior peso ate entao.
      {
        ponto_mais_viavel = i;
      }
  }
  return caminhos[ponto_mais_viavel]; //Retorna ponto de maior peso.
}

void CustomPlayer::exec() {
  if (!field || !frame || !robot) {
    return;
  }

  // TODO: here...
  // emit sendCommand(...);


  auto ball = frame->ball();
  auto allies = frame->allies();
  auto enemies = frame->enemies();
  auto center = field->center();


  // Ir ate a bola
  VSSMotion::GoToPoint GoToBall(ball.position());
  VSSRobotCommand GTB(GoToBall);

  //Ir ate o centro do campo
  VSSMotion::GoToPoint GoToCenter(field->center());
  VSSRobotCommand GTC(GoToCenter);

  //Ir ate o centro do proprio gol.
  VSSMotion::GoToPoint GoToGoal(field->allyGoalOutsideCenter());
  VSSRobotCommand GTG(GoToGoal);

  //3 funcoes que mandam ir ate um ponto generico, cada uma eh usada por apenas um robo.
  VSSMotion::GoToPoint GoToTarget0(alvo[0]);
  VSSRobotCommand GTT0(GoToTarget0);

  VSSMotion::GoToPoint GoToTarget1(alvo[1]);
  VSSRobotCommand GTT1(GoToTarget1);

  VSSMotion::GoToPoint GoToTarget2(alvo[2]);
  VSSRobotCommand GTT2(GoToTarget2);

  // giro horario
  VSSMotion::Spin spinClockwise(60, true);
  VSSRobotCommand SCW(spinClockwise);

  // giro anti-horario
  VSSMotion::Spin spinCounterClock(60, false);
  VSSRobotCommand SCC(spinCounterClock);


  for(int i = 0; i < 3; i++)
  //Esse for percorre sequencialmente os 3 robos executando o comportamento atual de cada um.
  {
    switch(estado[i])
    {
      case 0: //Planeja proxima acao.
        if(goleiro == i)
        {
          estado[i] = 3;
          tempo[i] = -1;
        }
        else
        {
          estado[i] = 2;
          tempo[i] = -1;
        }
        break;


      case 1: //Destrava robo ao detectar que ele passou muito tempo parado.

        emit sendCommand(vssNavigation.run((*allies.findById(i)), GTC));

        if(std::sqrt(std::pow(allies.findById(i)->velocity().x(), 2)
            + std::pow(allies.findById(i)->velocity().x(), 2)) >= 55
            || allies.findById(i)->distTo(field->center()) < 20)
        {
          //Se o robo chegar a uma distancia de 10 do centro ou atingir um velocidade maior
          //que 50, entao ele ja destravou e deve assumir outro estado.
          estado[i] = 0;
          tempo[i] = -1;
        }
        break;

      case 2: //Busca bola + desvia de obstaculos.
        alvo[i] = calculaPasso(i, allies.findById(i)->position(), passo, field, frame);
        switch(i)
        //Emissao do comando de perseguir alvo.
        {
          case 0: emit sendCommand(vssNavigation.run((*allies.findById(i)), GTT0)); 
            break;
          case 1: emit sendCommand(vssNavigation.run((*allies.findById(i)), GTT1));
            break;
          case 2: emit sendCommand(vssNavigation.run((*allies.findById(i)), GTT2));
            break;
        }

        break;

      case 3: //Vai ate gol aliado.

        emit sendCommand(vssNavigation.run((*allies.findById(i)), GTG));
        if(allies.findById(i)->distTo(field->allyGoalOutsideCenter()) <= 5)
        //Ao chegar em seu destino muda pra comportamento 4
        {
          estado[i] = 4;
          tempo[i] = -1; 
        }
        break;

      case 4: //Defesa tangente ao gol (acompanha a posicao y da bola).
        alvo[i][0] = field->allyGoalOutsideCenter().x();
        alvo[i][1] = max(
                      min(ball.y(), field->allyGoalOutsideTop().y()) , 
                      field->allyGoalOutsideBottom().y() );

        if(allies.findById(i)->distTo(alvo[i]) >= 4)
        {
          //Emissao do comando de perseguir alvo.
          switch(i)
          {
            case 0: emit sendCommand(vssNavigation.run((*allies.findById(i)), GTT0)); 
              break;
            case 1: emit sendCommand(vssNavigation.run((*allies.findById(i)), GTT1));
              break;
            case 2: emit sendCommand(vssNavigation.run((*allies.findById(i)), GTT2));
              break;
          }
        }
        
      break;
    }
    tempo[0]++; tempo[1]++; tempo[2]++;
    if(std::sqrt(std::pow(allies.findById(i)->velocity().x(), 2)
                 + std::pow(allies.findById(i)->velocity().x(), 2))
                    < 22 && goleiro != i)
    //Aqui fica a parte do codigo que checa se o robo esta preso em alguma coisa.
    //Se sua velocidade estiver bem proxima de 0, a variavel travado eh incrementada.
    {
      travado[i]++;
    }
    else
    //Se o robo atingir uma velocidade acima de 22 eh reconhecido que ele nao ta travado,
    //e a variavel travado eh zerada.
    {
      travado[i] = 0;
    }
    if(travado[i] >= 55 && travado[i] < 70)
    //Apartir de travado >= 55 reconhece-se que o robo travou e inicia-se a primeira tentativa
    //de destrava-lo, emitindo um comando de girar.
    {
      emit sendCommand(vssNavigation.run((*allies.findById(i)), SCW));
    }
    if(travado[i] > 80)
    //Se a girada nao for suficiente para destravar o robo, inicia-se uma rotina especial de
    //buscar o centro, representada pelo estado 1.
    {
      travado[i] = 0;
      estado[i] = 1;
      tempo[i] = 0;
    }
  }

//Essa parte atualiza os vetores vallVel e ballPosBuffer, que acabaram inutilizados na versao
//atual do codigo.
ballVel[0] = ball.x() - ballPosBuffer.x();
ballVel[1] = ball.y() - ballPosBuffer.y();
ballPosBuffer[0] = ball.x();
ballPosBuffer[1] = ball.y();


}

void CustomPlayer::receiveField(const Field& field) {
  shared->field = field;
}

void CustomPlayer::receiveFrame(const Frame& frame) {
  shared->frame = frame;
  runInParallel();
}

static_block {
  Factory::processing.insert<CustomPlayer>();
};
