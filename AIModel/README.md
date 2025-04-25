# AI Model Information: Tomato Detection

This directory contains information related to the AI model used for tomato detection in this project.

## Dataset

We used [Roboflow](https://roboflow.com/) to prepare the image dataset for this model. The dataset is organized into the standard splits:

- **Train:** Used for training the model.
- **Validate:** Used for tuning during training.
- **Test:** Used for final evaluation.

If you wish to retrain the model or inspect the data, you can download the complete dataset (ZIP archive) from the following link:

➡️ **[Download Tomato Dataset (MEGA.nz)](https://mega.nz/file/Gdd1yQ7Y#XgeF-OmfwyFg4fJsazg84tZfSf8YbSU1mU48wlkKB7c)**

## Model Access & Download

The final trained object detection model (`.onxx` format) is **not included** directly in this GitHub repository due to its large file size, which exceeds typical repository limits.

However, the model is publicly available and hosted on Ultralytics Hub. You can access and download it from the following link:

➡️ **[Ultralytics Hub: TomatoDetectBTV1](https://hub.ultralytics.com/models/G7BEsWD7K2GaELwWj6ka)**

**Action Required:** To run the parts of this project that rely on the AI model, please download the `.onxx` file from the link above and place it directly into this `AIModel` directory.

---

# AI モデル情報：トマト検出

このディレクトリには、本プロジェクトでトマト検出に使用される AI モデルに関する情報が含まれています。

## データセット

このモデルの画像データセットの準備には [Roboflow](https://roboflow.com/) を使用しました。データセットは標準的な分割に整理されています：

- **Train（訓練）:** モデルのトレーニングに使用。
- **Validate（検証）:** トレーニング中のチューニングに使用。
- **Test（テスト）:** 最終評価に使用。

モデルの再トレーニングやデータを確認したい場合は、以下のリンクから完全なデータセット（ZIP アーカイブ）をダウンロードできます：

➡️ **[トマトデータセット ダウンロード (MEGA.nz)](https://mega.nz/file/Gdd1yQ7Y#XgeF-OmfwyFg4fJsazg84tZfSf8YbSU1mU48wlkKB7c)**

## モデルへのアクセスとダウンロード

最終的にトレーニングされた物体検出モデル（`.onxx`形式）は、ファイルサイズが大きく、一般的なリポジトリの制限を超えるため、この GitHub リポジトリには直接**含まれていません**。

しかし、モデルは公開されており、Ultralytics Hub でホストされています。以下のリンクからアクセスし、ダウンロードすることができます：

➡️ **[Ultralytics Hub: TomatoDetectBTV1](https://hub.ultralytics.com/models/G7BEsWD7K2GaELwWj6ka)**

**必要な対応:** このプロジェクトの AI モデルに依存する部分を実行するには、上記のリンクから `.onxx` ファイルをダウンロードし、この `AIModel` ディレクトリに直接配置してください。
