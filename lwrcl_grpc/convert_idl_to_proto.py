import os
import re
import argparse
import subprocess

# IDLのデータ型をProtocol Buffersのデータ型にマッピングする辞書
type_mapping = {
    "boolean": "bool",
    "char": "int32",
    "short": "int32",
    "int8": "int32",
    "uint8": "uint32",
    "int16": "int32",
    "uint16": "uint32",
    "int32": "int32",
    "uint32": "uint32",
    "int64": "int64",
    "uint64": "uint64",
    "float": "float",
    "double": "double",
    "string": "string",
    "octet": "bytes",  # octetをbytesに変換

    # `unsigned`系の型
    "unsigned short": "uint32",
    "unsigned int": "uint32",
    "unsigned long": "uint32",
    "unsigned long long": "uint64",
    
    # `long`系の型
    "long": "int32",
    "long long": "int64",
}


def map_type(type_name, enum_names):
    """
    IDLの型名をProtoの型名に変換する。
    enum_names は検出したenumのリスト(またはset)で、 
    もし type_name がenum名に該当すれば <enum名>GRPC に置き換える。
    """

    # もしenum名なら "<enum名>GRPC" に変換
    if type_name in enum_names:
        return type_name + "GRPC"

    # `sequence<...>`の処理
    if type_name.startswith("sequence<"):
        inner_type = type_name[9:-1].strip()  # sequence<...> の ... 部分を取り出す
        return f"repeated {map_type(inner_type, enum_names)}"

    # スペースを含む型名などをチェック
    for key in sorted(type_mapping.keys(), key=len, reverse=True):
        # 例: "unsigned long long" などを先にマッチさせる
        if type_name == key:
            return type_mapping[key]

    # "::" を "." に置き換えた上で "GRPC" を付ける、という処理方針であればここで行う
    # ただし、type_mappingにない普通のクラス名などの場合だけにするのが無難。
    clean_type_name = re.sub(r'::', '.', type_name) + "GRPC"
    return type_mapping.get(type_name, clean_type_name)


def convert_enum(idl_content):
    """
    IDLのenum定義を解析して { enum名: [値1, 値2, ...], ... } の形で返す
    """
    enum_pattern = re.compile(r"enum\s+(\w+)\s*{([^}]*)}")
    enums = {}
    
    for match in enum_pattern.finditer(idl_content):
        enum_name = match.group(1).strip()
        # カンマ区切りで取り出し、末尾のセミコロンなど余計なものを除去
        enum_values_raw = match.group(2).strip().split(',')
        enum_values = []
        for v in enum_values_raw:
            v = v.strip()
            # 末尾にセミコロンがあれば除去
            v = re.sub(r';$', '', v)
            enum_values.append(v)
        enums[enum_name] = enum_values

    return enums


def convert_idl_to_proto(idl_content, include_files, proto_file_path):
    """
    IDL文字列をProto文字列に変換する。
    """
    proto_content = []
    struct_pattern = re.compile(r"struct\s+(\w+)\s*{")
    # フィールド解析用 (配列の '[]' に対応)
    field_pattern = re.compile(r"(sequence<[\w\s:]+>|[\w\s:]+)\s+(\w+)(\[\d*\])?\s*;")
    include_pattern = re.compile(r'#include\s+"([^"]+)"')
    
    # まずenumを取り出し
    enums = convert_enum(idl_content)
    # enum名リストをセット化
    enum_names = set(enums.keys())

    # Proto3とEmptyのインポート
    proto_content.append('syntax = "proto3";')

    # 一つ上のフォルダ名を取得してパッケージを設定
    parent_folder = os.path.basename(os.path.dirname(proto_file_path))
    grand_parent_folder = os.path.basename(os.path.dirname(os.path.dirname(proto_file_path)))
    proto_content.append(f'package {grand_parent_folder}.{parent_folder};')
    
    proto_content.append('import "google/protobuf/empty.proto";')

    # --- ここでenum定義をProtoに出力する ---
    # enumの数だけ "enum <Enum名>GRPC { ... }" を生成
    for enum_name, values in enums.items():
        proto_content.append(f"enum {enum_name}GRPC {{")
        # Proto3では最初の値を 0 にするのが一般的
        # IDLにはデフォルト値の概念がないので、そのまま並べる場合は
        # REVOLUTE=0, CONTINUOUS=1, ... のように振っていく
        for i, val in enumerate(values):
            proto_content.append(f"  {val} = {i};")
        proto_content.append("}\n")

    # 構造体が始まっているかどうかを管理する変数
    in_struct = False
    current_struct = None

    for line in idl_content.splitlines():
        line = line.strip()

        # include文を検出
        include_match = include_pattern.match(line)
        if include_match:
            included_file = include_match.group(1)
            include_files.add(included_file)
            proto_content.append(f'import "{os.path.basename(included_file).replace(".idl", ".proto")}";')
            continue

        # struct開始を検出
        struct_match = struct_pattern.match(line)
        if struct_match:
            struct_name = struct_match.group(1)
            struct_name_grpc = struct_name + "GRPC"
            proto_content.append(f"message {struct_name_grpc} {{")
            current_struct = struct_name_grpc
            in_struct = True
            field_index = 1
            continue

        if in_struct:
            # フィールド解析
            field_match = field_pattern.match(line)
            if field_match:
                field_type = field_match.group(1).strip()
                field_name = field_match.group(2).strip()
                array_size = field_match.group(3)

                # 配列の場合
                if array_size:
                    # "[]" のみなら repeated 扱い
                    if array_size == "[]":
                        proto_type = f"repeated {map_type(field_type, enum_names)}"
                    else:
                        # 固定長の場合もとりあえず repeated 扱いにする例
                        proto_type = f"repeated {map_type(field_type, enum_names)}"
                else:
                    proto_type = map_type(field_type, enum_names)

                proto_content.append(f"  {proto_type} {field_name} = {field_index};")
                field_index += 1
                continue

            # 構造体の終了 "}" を検出
            if line == "}":
                proto_content.append("}")
                in_struct = False
                continue

    # 最後までstructが閉じられていなかった場合は一応閉じる(念のため)
    if in_struct:
        proto_content.append("}")

    # サービス定義を追加
    if current_struct:
        proto_content.append(f"service {current_struct}Service {{")
        proto_content.append(f"  rpc SendGRPC({current_struct}) returns (google.protobuf.Empty);")
        proto_content.append("}")

    return "\n".join(proto_content)

# IDLファイルを読み込んでProtoファイルに変換
def convert_file(idl_file_path, proto_file_path, include_files):
    with open(idl_file_path, 'r') as idl_file:
        idl_content = idl_file.read()

    proto_content = convert_idl_to_proto(idl_content, include_files, proto_file_path)

    with open(proto_file_path, 'w') as proto_file:
        proto_file.write(proto_content)

    print(f"Converted {idl_file_path} to {proto_file_path}")

# IDLファイルをコンパイルする関数
def compile_proto(proto_file_path, error_files):
    # C++出力用に--cpp_outを設定
    protoc_command = [
        "/opt/grpc/bin/protoc", 
        "--proto_path=" + os.path.dirname(proto_file_path),
        "--proto_path=" + "/opt/grpc/include/",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../builtin_interfaces/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../diagnostic_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../diagnostic_msgs/srv",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../gazebo_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../gazebo_msgs/srv",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../geometry_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../lifecycle_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../lifecycle_msgs/srv",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../nav_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../nav_msgs/srv",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../pendulum_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../rcl_interfaces/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../rcl_interfaces/srv",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../sensor_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../sensor_msgs/srv",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../shape_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../std_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../std_msgs/srv",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../stereo_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../test_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../test_msgs/srv",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../tf2_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../tf2_msgs/srv",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../trajectory_msgs/msg",
        "--proto_path=" + os.path.dirname(proto_file_path) + "/../../visualization_msgs/msg",
        "--cpp_out=" + os.path.dirname(proto_file_path),
        "--grpc_out=" + os.path.dirname(proto_file_path),
        "--plugin=protoc-gen-grpc=/opt/grpc/bin/grpc_cpp_plugin",
        proto_file_path
    ]
    print(f"Compiling {proto_file_path}...")
    result = subprocess.run(protoc_command, capture_output=True, text=True)

    if result.returncode != 0:
        error_files.append((proto_file_path, result.stderr))

# 指定されたフォルダ以下にあるIDLファイルを順に処理
def process_folder(folder_path):
    include_files = set()
    proto_files = []
    error_files = []

    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith(".idl"):
                idl_file_path = os.path.join(root, file)
                proto_file_path = idl_file_path.replace(".idl", ".proto")
                convert_file(idl_file_path, proto_file_path, include_files)
                proto_files.append(proto_file_path)

    # includeされたIDLファイルも順に処理
    for include_file in include_files:
        include_path = os.path.join(folder_path, include_file)
        if os.path.exists(include_path):
            proto_file_path = include_path.replace(".idl", ".proto")
            if not os.path.exists(proto_file_path):  # 既に処理済みか確認
                convert_file(include_path, proto_file_path, set())  # 再帰処理しないため新しいセット
                proto_files.append(proto_file_path)

    # 変換したProtoファイルをコンパイル
    for proto_file in proto_files:
        compile_proto(proto_file, error_files)

    # コンパイルエラーを表示
    if error_files:
        print("Compilation errors:")
        for file_path, error_message in error_files:
            print(f"{file_path}:\n{error_message}")

if __name__ == "__main__":
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description="Convert IDL files to Proto files and compile them.")
    parser.add_argument("folder", type=str, help="Path to the folder containing IDL files")
    args = parser.parse_args()

    # フォルダパスの取得と処理開始
    folder_path = args.folder
    if os.path.isdir(folder_path):
        process_folder(folder_path)
    else:
        print(f"Error: {folder_path} is not a valid directory.")
