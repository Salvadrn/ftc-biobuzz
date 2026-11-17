#!/bin/bash
# Prueba la lógica del robot en la computadora, sin robot.
# Compila TeamCode contra los JAR reales del SDK y corre sim/Sim.java con tiempo virtual.
# Necesita un JDK 17:  brew install openjdk@17
set -e
cd "$(dirname "$0")"

SDK_VERSION=12.0.0
JSON_VERSION=20250517
if [ -d /opt/homebrew/opt/openjdk@17/bin ]; then
  export PATH=/opt/homebrew/opt/openjdk@17/bin:$PATH
fi
if ! javac -version >/dev/null 2>&1; then
  echo "No hay JDK. Instala uno con:  brew install openjdk@17" >&2
  exit 1
fi

mkdir -p .cache out
CP=""
fetch() {   # fetch <archivo local> <url>
  if [ ! -f "$1" ]; then
    echo "Descargando $(basename "$1")..."
    if ! curl -sfL -o "$1.tmp" "$2"; then
      echo "No se pudo descargar $2 (¿hay internet?)" >&2
      rm -f "$1.tmp"
      exit 1
    fi
    mv "$1.tmp" "$1"
  fi
}
for lib in RobotCore Hardware FtcCommon Vision; do
  jar=".cache/$lib-$SDK_VERSION.jar"
  if [ ! -f "$jar" ]; then
    fetch ".cache/$lib.aar" "https://repo1.maven.org/maven2/org/firstinspires/ftc/$lib/$SDK_VERSION/$lib-$SDK_VERSION.aar"
    unzip -oq ".cache/$lib.aar" classes.jar -d .cache
    mv .cache/classes.jar "$jar"
    rm ".cache/$lib.aar"
  fi
  CP="$CP:$jar"
done
fetch ".cache/json-$JSON_VERSION.jar" "https://repo1.maven.org/maven2/org/json/json/$JSON_VERSION/json-$JSON_VERSION.jar"
CP="$CP:.cache/json-$JSON_VERSION.jar"

rm -rf out/*
# stubs/: lo que en el robot da Android (android.util.Size) y una fábrica de LLResult
if ! javac -nowarn -source 8 -target 8 -d out -cp "$CP" \
    $(find ../robot/TeamCode/src/main/java -name "*.java") $(find stubs -name "*.java") *.java \
    > out/javac.log 2>&1; then
  grep -v "bootstrap class path\|^warning\|SuppressLint\|JsonAdapter\|^Note:" out/javac.log >&2
  echo "No compila." >&2
  exit 1
fi
java -cp "out$CP" Sim
