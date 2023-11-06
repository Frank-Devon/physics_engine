COMPILE="g++ -g -Wall -Werror *.cpp -lSDL2 -lSDL2_image -lSDL2_ttf -ldl -o a"
echo ${COMPILE}
eval $COMPILE
#./a
