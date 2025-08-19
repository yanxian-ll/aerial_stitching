if __name__ == "__main__":
    import argparse
    import os
    import time
    import shutil

    parser = argparse.ArgumentParser()
    parser.add_argument("--input_path", "-i", type=str, default="")
    parser.add_argument("--output_path", "-o", type=str, default="")
    parser.add_argument("--interval", "-t", type=float, default=1.0)
    args = parser.parse_args()

    # Record start time
    start_time = time.time()
    start_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_time))
    print(f"Start time: {start_time_str}")

    list_files = sorted([name for name in os.listdir(args.input_path) if name.endswith('JPG')])
    for i, file in enumerate(list_files):
        time.sleep(args.interval)
        shutil.copy2(os.path.join(args.input_path, file), os.path.join(args.output_path, file))
        print(f"Progress: {i+1} / {len(list_files)}")

    # Record end time
    end_time = time.time()
    end_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_time))
    print(f"End time: {end_time_str}")

    # Print total time taken
    total_time = end_time - start_time
    print(f"Total time taken: {total_time:.2f} seconds")
