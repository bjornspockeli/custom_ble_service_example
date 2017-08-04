/**
 * Copyright (c) 2017, Infineon Technologies AG ("contributors")
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA AND/OR CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/**
 * @file
 * @defgroup ifx_optiga_auth_example Infineon Authentication Example (main.c).
 * @{
 * @ingroup ifx_optiga
 * @brief Infineon OPTIGA Trust E authentication example application main file.
 *
 * This file contains the source code for a sample application
 * using Infineon OPTIGA Trust E hardware security module.
 *
 * For more information about the Infineon OPTIGA Trust E please visit:
 * https://www.infineon.com/cms/en/product/productType.html?productType=5546d4624f205c9a014f64f24f736ab3
 */


#include <stdio.h>
#include <stdlib.h> // calloc(), free
#include <stdbool.h> // true, false
#include "nrf_error.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "mbedtls/platform.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/sha256.h"
#include "ifx_optiga_command_library.h"


/**
 * Macro to log a line break and flush the logging buffer.
 */
#define LOG_LINEBREAK()                              \
    do {                                             \
        NRF_LOG_RAW_INFO("\r\n");                    \
        NRF_LOG_FLUSH();                             \
    } while (0)


// Prototypes
static int verify_signature(
    const uint8_t   *certificate,
    const uint32_t   certificate_len,
    const uint8_t   *asn_sig,
    const uint32_t   asn_sig_len,
    const uint8_t   *data,
    const uint32_t   data_len);

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info);

static void ecdsa_rs_to_asn1(
    const uint8_t  *r,
    uint32_t        r_len,
    const uint8_t  *s,
    uint32_t        s_len,
    uint8_t        *asn_sig,
    uint32_t       *asn_sig_len);


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    ret_code_t   err_code;
    uint8_t      rnd[16];
    uint8_t     *signature;
    uint32_t     signature_len;
    uint8_t     *certificate;
    uint32_t     certificate_len;
    uint8_t      asn1_sig[80];
    uint32_t     asn1_sig_len;

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    // Start internal LFCLK XTAL oscillator
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_RAW_INFO("OPTIGA Trust E authentication example\r\n");
    LOG_LINEBREAK();

    NRF_LOG_RAW_INFO("Opening OPTIGA Trust E application.\r\n");
    err_code = optiga_open_application();
    APP_ERROR_CHECK(err_code);
    LOG_LINEBREAK();

    NRF_LOG_RAW_INFO("Retrieving random numbers from OPTIGA Trust E:\r\n");
    err_code = optiga_get_random(sizeof(rnd), rnd);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_RAW_HEXDUMP_INFO(rnd, sizeof(rnd));
    LOG_LINEBREAK();

    NRF_LOG_RAW_INFO("Sign random numbers with protected private key inside OPTIGA Trust E:\r\n");
    err_code = optiga_set_auth_scheme();
    APP_ERROR_CHECK(err_code);
    err_code = optiga_sign(rnd, sizeof(rnd), &signature, &signature_len);
    // The signature value returned by optiga_sign is stored in a buffer inside the library.
    // Thus the signature value is only valid until a subsequent library call.
    // The call to ecdsa_rs_to_asn1 not just converts the signature format (to ASN.1), but also
    // copies it into a new buffer allocated locally in the main() function here.
    ecdsa_rs_to_asn1(&signature[0], 32, &signature[32], 32, asn1_sig, &asn1_sig_len);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_RAW_HEXDUMP_INFO(signature, signature_len);
    LOG_LINEBREAK();

    NRF_LOG_RAW_INFO("Retrieving public key certificate from OPTIGA Trust E.\r\n");
    err_code = optiga_read_certificate(&certificate, &certificate_len);
    APP_ERROR_CHECK(err_code);
    LOG_LINEBREAK();

    NRF_LOG_RAW_INFO("Signature verification with mbedTLS using the certificate's public key:\r\n");
    if (verify_signature(certificate, certificate_len, asn1_sig, asn1_sig_len, rnd, sizeof(rnd)) == 0)
    {
        NRF_LOG_RAW_INFO("OK - signature successfully verified!\r\n");
    }
    else
    {
        NRF_LOG_RAW_INFO("Signature invalid - verification failed.\r\n");
    }
    LOG_LINEBREAK();

    NRF_LOG_RAW_INFO("Infineon OPTIGA Trust E example application finished.\r\n");
    LOG_LINEBREAK();
    LOG_LINEBREAK();

    while (true)
    {
        nrf_pwr_mgmt_run();
    }
}


/**
 * Handles faults.
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_RAW_INFO("\r\nError.\r\n\r\n\r\n");
    NRF_LOG_FLUSH();
    while (true)
    {
        nrf_pwr_mgmt_run();
        // Do not automatically reset, otherwise we might trigger security events
        // on the OPTIGA Trust E due to to many subsequent sign calls
    }
}


/**
 * @brief Encodes the ECDSA signature components (r, s) in ASN.1 format.
 *
 * @param[in]   r            Component r of the ECDSA signature
 * @param[in]   r_len        Length of the r component of the ECDSA signature
 * @param[in]   s            Component s of the ECDSA signature
 * @param[in]   s_len        Length of the s component of the ECDSA signature
 * @param[out]  asn_sig      Buffer where the resulting ASN.1-encoded ECDSA signature will be copied into
 * @param[out]  asn_sig_len  Length of the actual data that was copied into the output buffer
 */
static void ecdsa_rs_to_asn1(
    const uint8_t  *r,
    uint32_t        r_len,
    const uint8_t  *s,
    uint32_t        s_len,
    uint8_t        *asn_sig,
    uint32_t       *asn_sig_len)
{
    uint32_t index;

    index = 0;

    asn_sig[index + 0] = 0x30;
    asn_sig[index + 1] =   70;
    index += 2;

    // R component
    asn_sig[index + 0] = 0x02;
    asn_sig[index + 1] = 0x20;
    if (r[0] & 0x80)
    {
        asn_sig[index + 1] += 1;
        asn_sig[index + 2] =  0;
        index++;
    }
    memcpy(&asn_sig[index + 2], &r[0], r_len);
    index += r_len + 2;

    // S component
    asn_sig[index + 0] = 0x02;
    asn_sig[index + 1] = 0x20;
    if (s[0] & 0x80)
    {
        asn_sig[index + 1] += 1;
        asn_sig[index + 2] =  0;
        index++;
    }
    memcpy(&asn_sig[index + 2], &s[0], s_len);
    index += s_len + 2;

    asn_sig[1] = index - 2; // Write actual length into ASN.1 data length field

    *asn_sig_len = index; // Return total length of ASN.1-encoded data structure
}


// Return 0 on successful verification, otherwise 1

/**
 * @brief Verifies an ECDSA signature using the public key contained in the certificate.
 *
 * @param[in] certificate X.509 certificate data which encode the public ECDSA verification key.
 * @param[in] certificate_len Length of the x.509 certificate data.
 * @param[in]
 *
 */
static int verify_signature(
    const uint8_t   *certificate,
    const uint32_t   certificate_len,
    const uint8_t   *asn_sig,
    const uint32_t   asn_sig_len,
    const uint8_t   *data,
    const uint32_t   data_len)
{
    int                     result;
    uint8_t                 digest[32]; // Stores a SHA-256 hash digest of 256 bits = 32 Byte
    mbedtls_x509_crt        x509_certificate;
    mbedtls_ecdsa_context   ecdsa_context;
    mbedtls_ecp_keypair    *ecp_keypair;

    // Setup memory management
    (void)mbedtls_platform_set_calloc_free(calloc, free);

    // Parse certificate and prepare verification context with public key
    mbedtls_x509_crt_init(&x509_certificate);
    result = mbedtls_x509_crt_parse(&x509_certificate, certificate, certificate_len);
    if (result != 0)
    {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }

    // Prepare ECDSA context with public key information from certificate
    mbedtls_ecdsa_init(&ecdsa_context);
    ecp_keypair = (mbedtls_ecp_keypair *) x509_certificate.pk.pk_ctx;
    if (mbedtls_ecdsa_from_keypair(&ecdsa_context, ecp_keypair) != 0)
    {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }

    // Calculate the SHA-256 hash digest from the to-be-signed data
    mbedtls_sha256(data, data_len, digest, 0);

    // Do the signature verification using context, hash, and signature
    result = mbedtls_ecdsa_read_signature(&ecdsa_context, digest, sizeof(digest), asn_sig, asn_sig_len);
    if (result != 0)
    {
        return -1; // Error - signature invalid
    }

    return 0; // Signature successfully verified
}

/** @} */
