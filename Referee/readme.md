このプログラムはロボカップ競技用のレフェリープログラムです．

* 注意事項
    * <https://github.com/RoboCupatHomeSim/RoboCupatHomeSim.git>はC:\SIGVerse\GitHubにCloneして下さい

* Refereeサービスのビルド手順
    1. C:\SIGVerse\GitHubに<https://github.com/SIGVerse/SIGService.git>をCloneする
    2. C:\SIGVerse\GitHub\SIGService\Windows.NET\SIGService_vc2010.slnを開く  
      （VC++コンパイラとライブラリをアップグレードしますといったウィンドウが表示された場合はキャンセルを選ぶ）
    3. プロジェクト->プロパティ->構成プロパティ->プラットフォーム ツールセットをVisual Studio 2010(v100)にする
    4. Releaseモードでビルドする  
      （ビルドをするための環境設定については<https://github.com/SIGVerse/SIGService/blob/master/README.md>参照）
    5. C:\SIGVerse\GitHub\RoboCupatHomeSim\Referee\RoboCupReferee_vs2010.slnを開く  
      （VC++コンパイラとライブラリをアップグレードしますといったウィンドウが表示された場合はキャンセルを選ぶ）
    6. プロジェクト->プロパティ->構成プロパティ->プラットフォーム ツールセットをVisual Studio 2010(v100)にする
    7. Releaseモードでビルドする

* Refereeサービスの使用方法
    - C:\SIGVerse\GitHub\RoboCupatHomeSim\Referee\Release\RoboCupReferee.sigをSIGViewerに登録して下さい