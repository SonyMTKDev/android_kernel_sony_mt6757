export USE_CCACHE=1
export CCACHE_EXEC=/usr/bin/ccache
ccache -M 25G
ccache -o compression=true

make ARCH=arm64 teak_defconfig
make ARCH=arm64 -j5
