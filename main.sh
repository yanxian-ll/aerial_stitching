#!/bin/bash

# 默认值
input_path=""
output_path=""
project_id=""
resolution="150"
seam_mode="1"
max_seam_width="256"
blend_mode="0"
single_mem_cpy=""
save_interval="10"
match_threshold="0.65"
idle_timeout="30"

# 解析输入参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--input_path)
            input_path="$2"
            shift 2
            ;;
        -o|--output_path)
            output_path="$2"
            shift 2
            ;;
        -p|--project_id)
            project_id="$2"
            shift 2
            ;;
        -r|--resolution)
            resolution="$2"
            shift 2
            ;;
        -s|--seam_mode)
            seam_mode="$2"
            shift 2
            ;;
        -w|--max_seam_width)
            max_seam_width="$2"
            shift 2
            ;;
        -b|--blend_mode)
            blend_mode="$2"
            shift 2
            ;;
        -m|--single_mem_cpy)
            single_mem_cpy="-m"
            shift
            ;;
        -n|--save_interval)
            save_interval="$2"
            shift 2
            ;;
        -t|--match_threshold)
            match_threshold="$2"
            shift 2
            ;;
        -x|--idle_timeout)
            idle_timeout="$2"
            shift 2
            ;;
        -h|--help)
            /usr/local/bin/fastcorrector -h
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# 检查必需参数
if [ -z "$input_path" ] || [ -z "$output_path" ] || [ -z "$project_id" ]; then
    echo "Error: --input_path, --output_path and --project_id are required."
    exit 1
fi


/usr/local/bin/fastcorrector \
    -i "$input_path" \
    -o "$output_path" \
    -p "$project_id" \
    -r "$resolution" \
    -s "$seam_mode" \
    -w "$max_seam_width" \
    -b "$blend_mode" \
    $single_mem_cpy \
    -n "$save_interval" \
    -t "$match_threshold" \
    -x "$idle_timeout"

# 获取程序返回值
exit_code=$?

# 返回程序的退出状态码
exit $exit_code
