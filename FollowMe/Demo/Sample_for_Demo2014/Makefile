#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include

OBJS = CheckPoint.so CheckPoint2.so FollowMeHuman.so FollowMeTestRobot.so Wall.so score.so FollowReferee.so Moderator.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/sigverse/commonlib -fPIC -shared -o $@ $<

clean:
	rm ./*.so
