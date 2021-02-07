-----
# xrotor設計ファイル 3Dモデル化スクリプト
-----

# 概要
xrotor_to_3dmodelは[xrotor](http://web.mit.edu/drela/Public/web/xrotor/)によるプロペラ設計ファイルから[fusion360](https://www.autodesk.co.jp/products/fusion-360/overview)で3Dモデルを生成するスクリプトです。fusion360のapiとpythonによって実装しています。以下デモ動画。

# 使い方

## セットアップ

[xrotor_to_design](https://github.com/melonTai/xrotor_to_design)と同様

## 入力

下記1~4は[xrotor_to_design](https://github.com/melonTai/xrotor_to_design)と同様
1. xrotor_restartfile
1. main_foil_path
1. sub_foil_path
1. hub_radius

- profile_interval：プロファイル(断面)の生成間隔
- center：xrotor_to_designのrib_centerと同様
- dat_amount：翼型の座標数

## mix foil

[xrotor_to_design](https://github.com/melonTai/xrotor_to_design)におけるリブをプロファイルに置き換えたもの

## option

チェックを入れた項目が実行される

- profile：プロファイル(断面)のスケッチ生成
- rail：レールのスケッチ(捻じれた物体のロフトに必要なガイドレール)
- loft：プロファイル間をロフトで繋げる

## **注意**

ロフトにガイドレールを適用する機能は比較的新しいため、スクリプト用に提供されていません。
したがって、railにチェックを入れると、ガイドレールのスケッチはされますが、ロフトには適用されません。
よって、**ガイドレールを適用したい場合は、loftのチェックを外し、適用とロフトを手動で行ってください。**
