#!/bin/bash

aws dynamodb update-item --table-name BuyList --key '{"Product": {"S": "Tea"}}' \
    --update-expression "ADD Quantity :val" \
    --expression-attribute-values '{":val":{"N": "1"}}' \
    --return-values ALL_NEW


