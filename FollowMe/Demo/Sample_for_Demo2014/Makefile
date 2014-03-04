#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include/sigverse

OBJS = CheckPoint.so CheckPoint2.so FollowMeHuman2.so FollowMeTestRobot_v2.so Wall.so score.so FollowReferee_v2.so Moderator.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

clean:
	rm ./*.so
