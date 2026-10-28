# shoulder_pan        : -0.350000
# shoulder_lift       : -0.670617
# elbow_flex          : +1.552391
# wrist_flex          : -0.232700
# wrist_roll          : +0.000721
# gripper             : -0.035000


# Gazebo に関節位置を送る関数
gz_joint() {
    docker compose exec -T gazebo gz topic \
        -t "/arm/cmd/$1" \
        -m gz.msgs.Double \
        -p "data: $2"
}

# 正面を向く
gz_joint shoulder_pan -0.35

# 手首の回転を戻す
gz_joint wrist_roll 0.0

# グリッパーを閉じる
gz_joint gripper -0.035

# アームを持ち上げる
gz_joint shoulder_lift -5
gz_joint elbow_flex 5

# 手首の角度を補正する
gz_joint wrist_flex 0.3

