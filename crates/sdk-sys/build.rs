fn main() {
    println!("cargo:rerun-if-changed=wrapper.hpp");
    println!("cargo:rerun-if-changed=../../vendor/scs_sdk/include");
    #[cfg(feature = "regenerate")]
    {
        let bindings = bindgen::Builder::default()
            .header("wrapper.hpp")
            .clang_args([
                "-x",
                "c++",
                "-std=c++11",
                "-fms-extensions",
                "-D_MSC_VER=1930",
                "-D_WIN64",
                "-I../../vendor/scs_sdk/include",
            ])
            .allowlist_type("scs_.*")
            .allowlist_var("SCS_.*")
            .blocklist_function(".*")
            .derive_default(true)
            .generate_comments(false)
            .layout_tests(true)
            .generate()
            .expect("generate bindings from pinned official SDK");
        bindings
            .write_to_file("src/bindings.rs")
            .expect("write bindings");
    }
}
