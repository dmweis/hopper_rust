fn main() {
    let mut proto_files = get_proto_files("proto/foxglove").unwrap();
    proto_files.extend_from_slice(&get_proto_files("proto/hopper").unwrap());

    prost_reflect_build::Builder::new()
        .descriptor_pool("crate::DESCRIPTOR_POOL")
        .compile_protos(&proto_files, &["./proto"])
        .unwrap();
}

fn get_proto_files(dir: &str) -> std::io::Result<Vec<String>> {
    let mut files = Vec::new();

    for entry in std::fs::read_dir(dir)?.flatten() {
        // If the entry is a file and its extension is 'proto'
        if entry.path().is_file() && entry.path().extension() == Some(std::ffi::OsStr::new("proto"))
        {
            // Add the file name to the list
            files.push(String::from(entry.path().to_string_lossy()));
        }
    }

    Ok(files)
}
