このノードは、PCと複数マイコン間の シリアル通信を統括的に管理 する．

interface_pkg::msg::SerialMsg型のsend_to_microトピックを送るとこのノードが自動的に指定されたマイコンにmsgを送ってくれる．
マイコンからのデータは同型のmicro_dataトピックで他ノードと共有される．

interface_pkg::msg::SerialMsgの中

| 変数名    | データ型       | 説明               |
| --------- | ---------- | ---------------- |
| `numbers` | `float[]`  | 浮動小数点数の配列        |
| `flags`   | `bool[]`   | 真偽値（ブール値）の配列     |
| `msg_str` | `string[]` | 文字列の配列           |
| `msg_id`  | `uint8`    | 符号なし8ビット整数（IDなど） |

