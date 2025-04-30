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

## Ubuntu Installation Note: ONNX Runtime

If you are running this code on Ubuntu, you might need to manually install the **ONNX Runtime** library.

The following resources provide detailed instructions. Please refer to them for the installation process (links last accessed: 2025-04-29):

- ➡️ **[Medium Article: ONNXRuntime integration with Ubuntu and CMake](https://medium.com/@massimilianoriva96/onnxruntime-integration-with-ubuntu-and-cmake-5d7af482136a)**
- ➡️ **[Stack Overflow: Setting up ONNX Runtime on Ubuntu 20.04 (C++ API)](https://stackoverflow.com/questions/63420533/setting-up-onnx-runtime-on-ubuntu-20-04-c-api)**

**⚠️ Crucial Step: Check Your OS Architecture**

It is **essential** to download and use the ONNX Runtime package that matches your system's architecture (e.g., `amd64` or `arm64`). Using the incorrect package will lead to errors.

When following the tutorials, pay close attention to any steps involving directory paths or download selections. Ensure you choose the files corresponding to your specific architecture.

For instance, if your system is `arm64`, you would need to select the `arm64` package or navigate into directories specific to it, like in this example path:

````bash
# Example path adjustment for an arm64 system
# Replace <version> with the specific ONNX Runtime version you are installing.
...
cd onnxruntime-linux-<version>/runtimes/linux-arm64/native/
...

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

## Ubuntuへのインストールに関する注意：ONNX Runtime

このコードをUbuntuで実行する場合、**ONNX Runtime** ライブラリを手動でインストールする必要があるかもしれません。

以下のリソースに詳細なインストール手順が記載されています。インストールプロセスについては、こちらを参照してください（最終アクセス日：2025年4月29日）：

* ➡️ **[Medium 記事: ONNXRuntime integration with Ubuntu and CMake](https://medium.com/@massimilianoriva96/onnxruntime-integration-with-ubuntu-and-cmake-5d7af482136a)**
* ➡️ **[Stack Overflow: Setting up ONNX Runtime on Ubuntu 20.04 (C++ API)](https://stackoverflow.com/questions/63420533/setting-up-onnx-runtime-on-ubuntu-20-04-c-api)**

**⚠️ 重要：OSアーキテクチャの確認**

ご使用のシステムのアーキテクチャ（例：`amd64` または `arm64`）に適合したONNX Runtimeパッケージをダウンロードして使用することが**不可欠**です。不適切なパッケージを使用するとエラーが発生します。

チュートリアルに従う際は、ディレクトリパスやダウンロードの選択に関する手順に特に注意してください。必ずご自身のアーキテクチャに対応するファイルを選択してください。

例えば、システムが `arm64` の場合、`arm64` パッケージを選択するか、以下のような `arm64` 固有のディレクトリに移動する必要があります：

```bash
# arm64 システム向けのパス調整例
# <version> はインストールする特定のONNX Runtimeバージョンに置き換えてください。
...
cd onnxruntime-linux-<version>/runtimes/linux-arm64/native/
...
````
