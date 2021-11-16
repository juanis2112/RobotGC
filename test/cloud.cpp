// Implements the cloud side of the protocol
// CHange all Alice to BOB?
#include "Cloud.h"
#include "emp-tool/execution/circuit_execution.h"
#include "DummySubsystem.h"

void print_init(Cloud *cloud, subSystem *subsystem, int k) {
  cout << "u" << 0 << ":  " << endl;
  for (int i = 0; i < cloud->sizeuk[0]; i++) {
    for (int j = 0; j < cloud->sizeuk[1]; j++) {
      cout << fixed << setprecision(5) << cloud->uk[i][j].reveal<double>(ALICE)
           << ", ";
    }
    cout << endl;
  }
  cout << endl << endl;
  cout << "z" << k << ":  " << endl;
  for (int i = 0; i < subsystem->sizeyk[0]; i++) {
    for (int j = 0; j < subsystem->sizeyk[1]; j++) {
      cout << subsystem->yk[i][j].reveal<double>(ALICE) << ", ";
    }
    cout << endl;
  }
  cout << endl << endl;
}

void print_rest( Cloud *cloud, subSystem *subsystem, int k) {
  cout << endl << endl;
  cout << "u" << k << ":  " << endl;
  for (int i = 0; i < cloud->sizeuk[0]; i++) {
    for (int j = 0; j < cloud->sizeuk[1]; j++) {
      cout << cloud->uk[i][j].reveal<double>(ALICE) << ", ";
    }
    cout << endl;
  }
  cout << endl << endl;
  cout << "z" << k << ":  " << endl;
  for (int i = 0; i < subsystem->sizeyk[0]; i++) {
    for (int j = 0; j < subsystem->sizeyk[1]; j++) {
      cout << subsystem->yk[i][j].reveal<double>(ALICE) << ", ";
    }
    cout << endl;
  }
  cout << endl << endl;
}

int main(int argc, char **argv) {
  int port, party;
  parse_party_and_port(argv, &party, &port);
  NetIO *io = new NetIO(party == ALICE ? nullptr : "127.0.0.1", port);
  setup_semi_honest(io, party);
  bool print = 0;

  
  
  
  

  subSystem *subsystem = new subSystem();
  Cloud *cloud = new Cloud();

  // client offline
  subsystem->initializeData();
  cloud->getInputs(subsystem->sizeTaylor, subsystem->sinCoeff, subsystem->cosCoeff, subsystem->sizeuk,
  subsystem->yd, subsystem->dFLi, subsystem->dFL, subsystem->sizeyk, subsystem->sizeyd);


  

  int k = 0;
 
  if (print) {
    print_init( cloud, subsystem, k);
  }

  cout << endl;
  // Control loop
  for (k = 0; k < 400; k++) {
    auto started = std::chrono::high_resolution_clock::now();
    cloud->computeuk(subsystem->yk);
    subsystem->measureState(cloud->uk);
    subsystem->computeyk();
    auto done = std::chrono::high_resolution_clock::now();
    cout << std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count()<<endl;
    if (print) {
      print_rest( cloud, subsystem, k+1);
    }
  }

  delete io;
  return 0;
}
