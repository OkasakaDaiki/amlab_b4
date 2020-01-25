# spl-meter-with-RPi : b4のラズパイのプログラム管理用のリポジトリ

*各プログラム説明*

testLCD_record2.py :   
testLCD_record5.py : MEMSマイクから拾った音圧データにA特性重み付け(たぶんできている)をしてRMSレベルをラズパイ付属の小型ディスプレイに表示

*問題点*

* ラズパイのストレージが厳しくなってきており, 追加でパッケージのインストールなどができない可能性が大きい  
* pip updateを実行してから, pipコマンドにおいてmainが見つかりませんというエラーが発生中(pip3では可能)

*課題*

* testLCD_record5.pyのプログラムは読めていないので, 本当にA特性重み付けがされていることを確認できていない  
* spl-meter-with-RPiパッケージを用いて周波数分析や周波数特性重み付けを実装したい(もしくは, サウンドプログラミング演習のプログラムをベースに同じ機能を実装したい)  
* testLCD_record2.pyからspl-meter-with-RPiの機能を呼び出したい()  
* testLCD_record3.pyで実現したいことは｢RMS計算値を取得して表示させる｣｢｣
