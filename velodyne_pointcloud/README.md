# velodyne_pointcloudの機能追加
本ブランチではvelodyne_pointcloudに対し、以下の機能を追加する。
* 数値計算・比較に半精度浮動小数点数や固定小数点数を用いる機能

### 動作環境

* Autoware (.iv/architecture proposal v0.9.1)
* Half-precision floating-point library 2.2.0
* libfixmath r64

### 事前準備

#### Half-precision floating-point library

1. http://half.sourceforge.net/ にアクセスし、ヘッダファイルをダウンロードする。
    * バージョン：2.2.0
    * ファイル名：half-2.2.0.zip

1. ダウンロードしたファイルを解凍し、解凍したディレクトリ内のinclude/half.hppを/usr/include/に移動する。

#### libfixmath

1. https://code.google.com/archive/p/libfixmath/downloads にアクセスし、ソースコードをダウンロードする。
    * バージョン：r64
    * ファイル名：libfixmath_r64.tar.gz

1. ダウンロードしたファイルを解凍し、解凍したディレクトリ内のMakefileにある`CPP_FLAGS`、`CC_FLAGS`、`LD_FLAGS`を以下のように修正してビルド時に`-fPIC`オプションを追加する。
    ```
    CPP_FLAGS = -O2 $(INC) -Wall -Wextra -c -fPIC
    CC_FLAGS  = -O2 $(INC) -Wall -Wextra -c -fPIC
    LD_FLAGS = -Wall -fPIC
    ```

1. 解凍したディレクトリ内で以下を実行する。
    ```
    $ make
    $ sudo cp libfixmath.a /usr/lib/
    $ sudo mkdir /usr/include/libfixmath
    $ sudo cp *.{h,hpp} /usr/include/libfixmath
    ```

### 環境設定

以下の各ノードに対する環境設定について記載する。必要に応じて、各ノードを起動するlaunchファイルで以下のパラメータ値を設定し、実行する。

#### cloud_node/cloud_nodelet

|名前|型|説明|既定値|
|:---|:---|:---|:---|
|velodyne_convert_implement_type|int|convert処理の実装方法を指定する。以下の値を指定できる。<br>0：処理中の数値に単精度浮動小数点数を用いる。<br>1：処理中の数値に単精度浮動小数点数を用いる。<br>2：処理中の数値に半精度浮動小数点数を用いる。<br>3：処理中の数値に固定小数点数（整数部16bit、小数部16bit）を用いる。|0|

* 設定例
```
<param name="velodyne_convert_implement_type" value="3" />
```

#### interpolate_node/interpolate_nodelet

|名前|型|説明|既定値|
|:---|:---|:---|:---|
|velodyne_interpolate_implement_type|int|interpolate処理の実装方法を指定する。以下の値を指定できる。<br>0：tf2の関数を実行する。処理中の数値は単精度浮動小数点数を用いる。<br>1：tf2関数内部の処理をinterpolate_node/interpolate_nodeletで行う。処理中の数値は単精度浮動小数点数を用いる。<br>2：tf2関数内部の処理をinterpolate_node/interpolate_nodeletで行う。処理中の数値は半精度浮動小数点数を用いる。<br>3：tf2関数内部の処理をinterpolate_node/interpolate_nodeletで行う。処理中の数値は固定小数点数（整数部16bit、小数部16bit）を用いる。|0|

* 設定例
```
<param name="velodyne_interpolate_implement_type" value="2" />
```
