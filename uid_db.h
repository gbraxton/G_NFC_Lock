/*
 * uid_db.h
 *
 * Created: 12/5/2013 6:22:15 PM
 *  Author: gbraxton
 */ 


#ifndef UID_DB_H_
#define UID_DB_H_

#include "mfrc522.h"

#define MAX_NUMBER_VALID_KEYS 5
#define MASTER_UID "24598D0D"

// temporary pointer to hold the last uid read
char tag_id[9] = {0, 0, 0, 0, 0, 0, 0, 0, '\0'};
char* tmp_char_ptr;

// array of valid key tags
char* valid_tag_db[MAX_NUMBER_VALID_KEYS] = {NULL};
// current number of valid keys in db
unsigned char numValidKeys = 0;

void storeTagID(){
    PICC_ReadCardSerial();
    unsigned char i;
    for(i = 0; i < 4; i++){
        sprintf(tag_id + (i*2), "%X", (uid.uidByte[i] & 0xF0) >> 4);
        sprintf(tag_id + (i*2 + 1), "%X", (uid.uidByte[i]) & 0x0F);
    }
}

unsigned char isTagDBEmpty(){
    return (numValidKeys == 0);
}

unsigned char isTagDBFull(){
    return (numValidKeys == MAX_NUMBER_VALID_KEYS);
}

unsigned char isMaster(char* key){
    if(strcmp(key, MASTER_UID) == 0){
        return 1;
    } else {
        return 0;
    }
}

signed char add_valid_tag(char* key){
    if(!isTagDBFull()){
        tmp_char_ptr = malloc(sizeof(char) * 9);
        strcpy(tmp_char_ptr, key);
        valid_tag_db[numValidKeys] = tmp_char_ptr;
        numValidKeys++;
        return 1;
    } else {
        return -1;
    }
}

signed char search_valid_tag(char* key){
    if(isTagDBEmpty()){
        return -1;
    }
    unsigned char i;
    for(i = 0; i < numValidKeys; i++){
        if(strcmp(key, valid_tag_db[i]) == 0){
            return i;
        }
    }
    return -1;
}

signed char remove_valid_tag(unsigned char db_index){
    if(db_index < 0 || db_index >= numValidKeys){
        return -1;
    } else {
        unsigned char i;
        for(i = db_index; i < numValidKeys; i++){
            valid_tag_db[i] = valid_tag_db[i+1];
        }
        numValidKeys--;
    }
    return 1;
}

#endif /* UID_DB_H_ */