'use strict';
var AWS = require("aws-sdk");
var sns = new AWS.SNS();

exports.handler = (event, context, callback) => {

    event.Records.forEach((record) => {
        console.log('Stream record: ', JSON.stringify(record, null, 2));
        
        if (record.eventName == 'INSERT') {
	    var product = JSON.stringify(record.dynamodb.NewImage.Product.S);
	    var amount = JSON.stringify(record.dynamodb.NewImage.Quantity.N);
            var params = {
                Subject: 'ELEC424 Amazon Dot', 
                Message: 'User has placed order for:' + product + '. \n\n' + 
		'User has ordered ' + amount + 'units for order.\n\n',
                TopicArn: 'arn:aws:sns:us-east-1:165657364714:purchaseTopic'
            };
            sns.publish(params, function(err, data) {
                if (err) {
                    console.error("Unable to send message. Error JSON:", JSON.stringify(err, null, 2));
                } else {
                    console.log("Results from sending message: ", JSON.stringify(data, null, 2));
                }
            });
        }
    });
    callback(null, `Successfully processed ${event.Records.length} records.`);
};   
