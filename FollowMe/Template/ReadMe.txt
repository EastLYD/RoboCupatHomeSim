作成日　　2014/2/6

SIGVerseを用いたRoboCup@Homeシミュレーションのためのレフェリーコントローラ(とテスト用ロボットコントローラ)

動作環境

SIGVerseのVer.        2.2.0
ViewerのVer.          2.2.0

サーバ側　　OS:ubuntu 12.04 LTS 32bit
            プロセッサ：Intel Core i5 CPU 750@2.67GHz*4
            メモリ: 4GB

Viewer側　　OS : Windows 7 Enterprise
            プロセッサ : Intel Core i5 CPU 650@3.20GHz
            メモリ : 4GB

ディレクトリの説明

・FollowTest
開発を行ったディレクトリ。sigcreateで作成したディレクトリにFollowTestの中身をコピペすれば動作すると思います。(ただしman_nii_v2_mikami.x3d,Man-nii_mikami.xmlは入っていない)

・shape
環境中に登場する壁や人の形状ファイル。
man_nii_v2_mikami.x3dは sigverse-2.2.0/share/sigverse/data/shapeにいれて動作させました。

・so
soファイルです。世界ファイルでコントローラをアタッチしたいエンティティにこれらのファイル名を指定してください。

・source
ソースコードいれてます。

・txt
ロボットやレフェリーが移動するときに読み込む経路のノードを記述してます。
0,0,0,1という記述をしています。最初の3つはXYZ座標,最後の一つはチェックポイント用の値です。

・xml
世界ファイルです。
Man-nii_mikami.xmlは sigverse-2.2.0/share/sigverse/data/xmlにいれて動作させました。

・texture
テクスチャファイルを入れています。
man_c01_mikami.jpgはサーバ側でなくビューワ側のPCに入れる必要があります。
場所は　Program Files(x86)/SIGViewer_2.2.0/SIGViewer/media/materials/texturesです


プログラムの説明
・CheckPoint
チェックポイントは基本的にCleanUpのTrashBoxとほぼ同様の動きをします。自身とエンティティ（今回はロボットのみ）の位置関係を計測し、その結果によりメッセージをscoreに送っています。

・FollowMeHuman2
このプログラムはレフェリーからメッセージを受け取ると一定の距離を前進します。

・FollowMeTestRobot_v2
このロボットコントローラはテスト用のコントローラであらかじめnodeファイルから座標を読み込み、レフェリーに完璧に追従することができます。また、各チェックポイントでレフェリーにメッセージをおくります。
現在のプログラムでは、"elevator"というメッセージをエレベータ以外の場所でも発言しているため、減点されます。この機能を切るには239行目と242行目をコメントアウトしてください。（elseのコードブロックの中のsendMsgをコメントアウト）

・Wall
このプログラムはレフェリーからメッセージを受け取るとドアの開閉をおこないます。

・FollowReferee_v2
レフェリーです。レフェリーはあらかじめ移動するための座標をnodeファイルから読み込みます。なお最初はstartのメッセージを受け取ることで動き出します。レフェリーは各nodeに達したとき、ロボットが一定の距離より離れているとロボットを待ちます。また、各チェックポイントではそれに応じたメッセージを各エンティティに送信します。

・score
FollowReferee_v2やCheckPointからのメッセージを受け取りポイントを加算するプログラムです。ロボットが衝突すると強制的に０点になります。


動画の説明

・FollowAllClear
ロボットがレフェリーを追従します。ロボットは完璧にタスクをこなすようにあらかじめチェックポイントの座標などを知っています。
なお、レフェリーが体の向きを変える時に変な方向を向くというバグがあります。