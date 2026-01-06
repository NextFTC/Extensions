plugins {
    kotlin("android")
    id("com.android.library")
}

android {
    namespace = "dev.nextftc.extensions.fateweaver"
    compileSdk = 35

    defaultConfig {
        minSdk = 24
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    kotlinOptions {
        jvmTarget = "1.8"
        freeCompilerArgs += "-Xjvm-default=all"
    }

    publishing {
        singleVariant("release")
    }
}

dependencies {
    implementation(libs.bundles.nextftc)
    implementation(libs.bundles.ftc)
    implementation(libs.fateweaver)
}

description =
    "Support for using FateWeaver with NextFTC."

nextFTCPublishing {
    displayName = "NextFTC Extensions - FateWeaver"
    logoPath = "../assets/logo-icon.svg"
    version = property("versions.fateweaver") as String
}