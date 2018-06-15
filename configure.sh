apt-get update && apt-get upgrade
apt-get -y install pkg-config
apt-get -y install g++ cmake libxmu-dev libxi-dev protobuf-compiler libprotobuf-dev libboost-all-dev freeglut3 freeglut3-dev
apt-get -y install g++ cmake libzmqpp3 libzmqpp-dev protobuf-compiler libprotobuf-dev libboost-all-dev libbullet-dev
curl -sSf https://static.rust-lang.org/rustup.sh | sh
cargo install protobuf
apt-get install libjsoncpp-dev