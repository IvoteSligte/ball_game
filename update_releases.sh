cargo build --release --target=x86_64-pc-windows-gnu
zip -r releases/windows.zip assets
zip -j releases/windows.zip target/x86_64-pc-windows-gnu/release/ball_game.exe

cargo build --release
cd target/release
tar --exclude='assets/save_data' -zcvf ../../releases/linux.tar.gz ../../assets ball_game