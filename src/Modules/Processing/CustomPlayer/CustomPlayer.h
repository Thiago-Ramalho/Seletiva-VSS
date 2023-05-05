#ifndef PROJECT_UNIFICATION_CUSTOMPLAYER_H
#define PROJECT_UNIFICATION_CUSTOMPLAYER_H

#include "Modules/Modules.h"
#include "Modules/Processing/ProcessingUtils/ProcessingUtils.h"

class CustomPlayer : public Processing {
 public:
  CustomPlayer(int index, QThreadPool* threadPool);

 protected:
  void buildParameters(Parameters::Handler& parameters) override;
  void connectModules(const Modules* modules) override;
  void init(const Modules* modules) override;
  void update() override;
  void exec() override;

 private:
  struct Args {};
  Args args;

  struct Shared {
    SharedOptional<Frame> frame;
    SharedOptional<Robot> robot;
    SharedOptional<Field> field;
    SharedValue<QSet<Qt::Key>> keys;
  };
  SharedWrapper<Shared, std::mutex> shared;

  std::optional<Field> field;
  std::optional<Frame> frame;
  std::optional<Robot> robot;

  SSLNavigation sslNavigation;
  VSSNavigation vssNavigation;

  

  int goleiro = 2, zagueiro = 1, atacante = 0; //ID do robo que estah no momento ocupando cada funcao.

  int estado[3] = {0, 0, 0}; //Vetor que armazena estado dos robos: {robo0, robo1, robo2}.
  int tempo[3] = {0, 0, 0}; //Armazena o tempo desde a ultima mudanca de estado de cada robo.
  int travado[3] = {0,0,0};

  Point ballPosBuffer = {0, 0}; //NAO UTILIZADO Armezena a posicao da bola ao final de cada iteracao.
  Point ballVel = {0, 0}; //NAO UTILIZADO Estima a velocidade da bola a partir do PosBuffer.
  Point ballVelNorm = {0,0}; //NAO UTILIZADO Versao normalizada do ballVel, indica apenas a sentido do movimento da bola.

  Point alvo[3] = {{0,0}, {0,0}, {0,0}}; //Pontos genericos que serah passado como parametro para um GoToPoint.
  
  std::vector<Point> percurso[3] = {{}, {}, {}}; //NAO UTILIZADO
  
  float passo = 8; //Utilizado na funcao calculaPasso, o robo no estado 2 sempre estara
  //perseguindo um ponto a uma distancia de 'passo' da posicao do robo.


 private slots:
  void receiveField(const Field& field);
  void receiveFrame(const Frame& frame);
};

#endif // PROJECT_UNIFICATION_CUSTOMPLAYER_H
