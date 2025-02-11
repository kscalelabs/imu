fn main() {
    // Tell cargo to look for Python libraries in the conda env
    println!("cargo:rustc-link-search=/home/kuwajerw/anaconda3/envs/zdemos/lib");
    
    // Link against Python library (name might be python3.x or python3.x.so)
    println!("cargo:rustc-link-lib=python3.11");
} 