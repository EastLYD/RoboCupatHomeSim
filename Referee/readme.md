このプログラムはロボカップ競技用のレフェリープログラムです．

*注意事項
    *https://github.com/RoboCupatHomeSim/RoboCupatHomeSim.gitはC:\SIGVerse\GitHubにCloneして下さい

*Refereeサービスのビルド手順
    1.C:\SIGVerse\GitHubにhttps://github.com/SIGVerse/SIGService.gitをCloneする
    2.C:\SIGVerse\GitHub\SIGService\Windows.NET\SIGService_vc2010.slnを開く（VC++コンパイラとライブラリをアップグレードしますといったウィンドウが表示された場合はキャンセルを選ぶ）
    3.プロジェクト->プロパティ->構成プロパティ->プラットフォーム ツールセットをVisual Studio 2010(v100)にする
    4.Releaseモードでビルドする（ビルドをするための環境設定についてはhttps://github.com/SIGVerse/SIGService/blob/master/README.md参照）
    5.C:\SIGVerse\GitHub\RoboCupatHomeSim\Referee\RoboCupReferee_vs2010.slnを開く（VC++コンパイラとライブラリをアップグレードしますといったウィンドウが表示された場合はキャンセルを選ぶ）
    6.プロジェクト->プロパティ->構成プロパティ->プラットフォーム ツールセットをVisual Studio 2010(v100)にする
    7.Releaseモードでビルドする

*Refereeサービスの使用方法
    -C:\SIGVerse\GitHub\RoboCupatHomeSim\Referee\Release\RoboCupReferee.sigをSIGViewerに登録して下さい