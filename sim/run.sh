#!/bin/bash
# Prueba la lógica de apuntado en la computadora, sin robot.
# Compila TeamCode contra los JAR reales del SDK y corre sim/Sim.java.
# Necesita un JDK 17:  brew install openjdk@17
set -e
cd "$(dirname "$0")"

SDK_VERSION=12.0.0
if [ -d /opt/homebrew/opt/openjdk@17/bin ]; then
  export PATH=/opt/homebrew/opt/openjdk@17/bin:$PATH
fi

mkdir -p .cache out
CP=""
for lib in RobotCore Hardware FtcCommon; do
  jar=".cache/$lib-$SDK_VERSION.jar"
  if [ ! -f "$jar" ]; then
    echo "Descargando $lib $SDK_VERSION..."
    curl -sfL -o ".cache/$lib.aar" "https://repo1.maven.org/maven2/org/firstinspires/ftc/$lib/$SDK_VERSION/$lib-$SDK_VERSION.aar"
    unzip -oq ".cache/$lib.aar" classes.jar -d .cache
    mv .cache/classes.jar "$jar"
    rm ".cache/$lib.aar"
  fi
  CP="$CP:$jar"
done

rm -rf out/*
javac -nowarn -source 8 -target 8 -d out -cp "$CP" \
  $(find ../robot/TeamCode/src/main/java -name "*.java") Sim.java 2>&1 \
  | grep -v "bootstrap class path\|warning\|SuppressLint\|JsonAdapter\|^Note:" || true
java -cp "out$CP" Sim
