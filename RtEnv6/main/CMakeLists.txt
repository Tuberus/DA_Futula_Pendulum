
idf_component_register( SRCS "MotorWmpuTest.cpp"
                             
  INCLUDE_DIRS "." 
               "../components/dsp" 
               "../components/RtEnvHL/commu" 
               "../components/RtEnvHL/core" 
  PRIV_REQUIRES RtEnvHL )
