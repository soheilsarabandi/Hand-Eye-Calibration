function startup()
   disp('Code associated with the paper:');
   disp(' ');
   disp('   Hand-Eye Calibration Made Easy through a Closed-Form Two-Stage Method');
   disp('by');
   disp('   S. Sarabandi, J. M. Porta, and F. Thomas');
   disp('  ');
   disp('Use the functions in the ''Scripts'' folder to replicate the results in the paper:');
   disp(' ');
   disp('     - Table I-Column 2:      NoiselessDataRandom(1);');
   disp('     - Table I-Column 3:      NoiselessDataRandom(2);');
   disp('     - Table I-Column 4:      NoiselessDataRandom(3);');
   disp('     - Table I-Column 5:      NoiselessDataRandom(4);');
   disp('     - Table I-Column 6:      NoiselessDataRandom(5);');
   disp('     - Figure 2-top row:      NoisyTranslationRandom;');
   disp('     - Figure 2-bottom row:   NoisyRotationRandom;');
   disp('     - Figure 3:              ExecutionTime;');
   disp('     - Table II:              Experiment2(''RealDataSet'');');
   disp('     - Table III:             Experiment(''RealDataSet'');');
   disp(' ');
   disp('See the help for each script for more details and for complementary experiments.');
   
   addpath('./');
   addpath('./RealDataSet');
   addpath('./Methods');
   addpath('./Scripts');
   addpath('./Utils');
