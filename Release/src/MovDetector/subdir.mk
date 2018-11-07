################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/MovDetector/BGFGTrack.cpp \
../src/MovDetector/Kalman.cpp \
../src/MovDetector/MOG3.cpp \
../src/MovDetector/MSTracker.cpp \
../src/MovDetector/MatchTracker.cpp \
../src/MovDetector/MovDetector.cpp \
../src/MovDetector/mvdectInterface.cpp \
../src/MovDetector/postDetector.cpp \
../src/MovDetector/psJudge.cpp 

C_SRCS += \
../src/MovDetector/vibe-background-sequential.c 

OBJS += \
./src/MovDetector/BGFGTrack.o \
./src/MovDetector/Kalman.o \
./src/MovDetector/MOG3.o \
./src/MovDetector/MSTracker.o \
./src/MovDetector/MatchTracker.o \
./src/MovDetector/MovDetector.o \
./src/MovDetector/mvdectInterface.o \
./src/MovDetector/postDetector.o \
./src/MovDetector/psJudge.o \
./src/MovDetector/vibe-background-sequential.o 

C_DEPS += \
./src/MovDetector/vibe-background-sequential.d 

CPP_DEPS += \
./src/MovDetector/BGFGTrack.d \
./src/MovDetector/Kalman.d \
./src/MovDetector/MOG3.d \
./src/MovDetector/MSTracker.d \
./src/MovDetector/MatchTracker.d \
./src/MovDetector/MovDetector.d \
./src/MovDetector/mvdectInterface.d \
./src/MovDetector/postDetector.d \
./src/MovDetector/psJudge.d 


# Each subdirectory must supply rules for building sources it contributes
src/MovDetector/%.o: ../src/MovDetector/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -I/usr/include/opencv -I/usr/include/GL -I../include -I../src/MovDetector -I../src/cr_osa/inc -O3 -Xcompiler -fPIC -Xcompiler -fopenmp -ccbin aarch64-linux-gnu-g++ -gencode arch=compute_52,code=sm_52 -m64 -odir "src/MovDetector" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -I/usr/include/opencv -I/usr/include/GL -I../include -I../src/MovDetector -I../src/cr_osa/inc -O3 -Xcompiler -fPIC -Xcompiler -fopenmp --compile -m64 -ccbin aarch64-linux-gnu-g++  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/MovDetector/%.o: ../src/MovDetector/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -I/usr/include/opencv -I/usr/include/GL -I../include -I../src/MovDetector -I../src/cr_osa/inc -O3 -Xcompiler -fPIC -Xcompiler -fopenmp -ccbin aarch64-linux-gnu-g++ -gencode arch=compute_52,code=sm_52 -m64 -odir "src/MovDetector" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -I/usr/include/opencv -I/usr/include/GL -I../include -I../src/MovDetector -I../src/cr_osa/inc -O3 -Xcompiler -fPIC -Xcompiler -fopenmp --compile -m64 -ccbin aarch64-linux-gnu-g++  -x c -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


