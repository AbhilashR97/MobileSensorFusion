MATLAB_FILES= src/matlab/dQqdq.m \
              src/matlab/q2euler.m \
              src/matlab/filterTemplate.m \
              src/matlab/mu_normalizeQ.m \
              src/matlab/OrientationView.m \
              src/matlab/Qq.m \
              src/matlab/showIP.m \
              src/matlab/Somega.m \
              src/matlab/Sq.m \
              src/matlab/startup.m

JAR_FILES= bin/sensordata.jar

all : sensorfusion.tar.gz

sensorfusion.tar.gz : ${MATLAB_FILES} ${JAR_FILES}
	tar czf $@ -C ${PWD}/src/matlab $(notdir ${MATLAB_FILES}) -C ${PWD}/bin $(notdir ${JAR_FILES})

.PHANTOM : clean

clean :
	${RM} *~ sensorfusion.tar.gz
