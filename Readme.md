<!-- [GLFM](https://docs.gitlab.com/ee/user/markdown.html)   -->

# Readme.md

## 概要

双葉電子工業製の産業用サーボのコマンド式サーボを使用したロボットを ros2_control から制御するための Hardware モジュール。

A ros2_control hardware module designed to control robots using industrial-grade servos of the command type, manufactured by Futaba Corporation.

現在は４つのサーボモジュールを順に ID.1, 2, 3, 4 となるようにハードコードされている。[TODO]


## 設計メモ

 - 「１つのチェイン」は「１つのシリアル配線に連なる複数のサーボモジュール」
 - 部位単位で hardware_fcs のインスタンスを作る想定。
 - １つの部位が複数のチェインで構成されるケースも想定。
 - [TODO] シリアルポートの指定方法に USB デバイスの idSerial も指定できるようにする
 - mcidx は 16bit データで上位 8bit は fcs_command_positions_[] のインデスクに対応する。
 - mcidx の下位 bit は各サーボモジュールの ROM に焼き込まれた ID 番号に対応する。
 - chain_index_list_[] は軸数分あり、部位を構成する軸に対応するチェインオブジェクトと mcidx のペアで構成されている。
   - chain_index_list_[] の設定順に並ぶことを想定し、データの方に fcs_command_positions_[] のインデスクを含めるために mcidx で工夫した。
   - 当初, ros2_control の hardrware のインタフェース構成がわかっておらず、迷走した名残なので、std::tuple とか構造体とかの実装のほうがよさげ。[TODO]
   
 - シリアル通信による状態収集、コマンド送信に比較的時間がかかるため、read()/write() のサイクルとは別に Reading/Read/Writing/Written の４状態を管理している。
 - read() 呼び出し時に Read であれば、内部のデータバッファから状態インタフェースへデータをコピーする。コピーが終わったら Writing へ遷移する。
 - write() 呼び出し時に Writing であれば、コマンドインタフェースの指令値を読み出して送信データを用意し、送信する。送信したら Written へ遷移して、notify() する。
 - 別スレッドでは Written 状態になるまで待機し、Written 状態になったら Reading へ遷移して、各チェインに対して状態読み出し通信を行う。全軸分の通信が終わったら Read 状態に遷移し、待機に戻る。
 
 - 現在は４つのサーボモジュールを順に ID.1, 2, 3, 4 となるようにハードコードされている。
   - [TODO] 設定で変更できるようにする
   - [TODO] パラメータ化して on_configure() で設定を反映するようにする


on_configure()
  - シリアルポート(複数) を開いてセットアップ
  - [TODO] 軸名確認、ポート名とサーボインデスクに対する軸名のマップ作成
    - parameter: { port_name_0 : { index_0: joint_name_0, index_1: joint_name_1, ...}, port_name_1 : { .... } }

on_activate()
  - Torque Enable (サーボオン)
  - サーボオフ中に外力で動かした時用の対策に、「指令値を現在値で上書きしてから Torque Enable を送る」

on_deactivate()
  - Torque Disable (サーボオフ)
 
on_cleanup()
  - サーボオフ（[TODO] 不要なので削除？）
  - シリアルポートクローズ

周期 write
  - [TODO]

周期 read
  - [TODO]


## ファイル構成




```
.
├── CMakeLists.txt
├── Readme.md
├── hardware_fcs.cc
├── hardware_fcs.xml
├── include
│   └── hardware_fcs
│       ├── emesg_stderr.h
│       ├── futaba_command_servo.h
│       ├── hardware_fcs.h
│       └── usb_serial.h
├── package.xml
└── tests
    ├── 00memo.org
    ├── build.sh
    ├── test3.cc
    ├── test4.cc
    ├── test5.cc
    ├── test6.cc
    ├── test7.cc
    ├── test8.cc
    ├── test9.cc
    ├── testA.cc
    ├── testB.cc
    └── testC.cc
```

