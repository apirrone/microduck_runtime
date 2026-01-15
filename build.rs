fn main() {
    // Only add C++ linking for the target platform, not for build scripts
    if std::env::var("TARGET").unwrap().contains("aarch64") {
        println!("cargo:rustc-link-lib=stdc++");
    }
}
