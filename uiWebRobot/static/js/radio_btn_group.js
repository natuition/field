function checked(event){
    if(event.target.checked){
        for(let elem of document.querySelectorAll('.RadioButtonGroup')){
            if(elem.contains(event.target)){
                for(let input of elem.getElementsByTagName("input")){
                    if(input != event.target){
                        input.checked = false;
                    }
                }
            }
        }
    }else event.preventDefault();
}

for(let elem of document.querySelectorAll('.RadioButtonGroup')){
    let inputs = elem.getElementsByTagName("input");
    for(let input of inputs){
        input.addEventListener("click",checked);
    }
}
